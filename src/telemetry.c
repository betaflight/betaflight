/* 
 * FrSky Telemetry implementation by silpstream @ rcgroups
 */
#include "board.h"
#include "mw.h"

#define CYCLETIME       200

static void sendDataHead(uint8_t id)
{
    uartWrite(0x5E);
    uartWrite(id);
}

static void sendTelemetryTail(void)
{
    uartWrite(0x5E);
}

static void serialize16(int16_t a)
{
    uint8_t t;
    t = a;
    uartWrite(t);
    t = a >> 8 & 0xff;
    uartWrite(t);
}

static void sendAccel(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        sendDataHead(0x24 + i);
        serialize16(((float)accSmooth[i] / acc_1G) * 1000);
    }
}

static void sendBaro(void)
{
    sendDataHead(0x10);
    serialize16(EstAlt / 100);
    sendDataHead(0x21);
    serialize16(EstAlt % 100);
}

static void sendTemperature1(void)
{
    sendDataHead(0x02);
    serialize16(telemTemperature1 / 10);
}

static void sendTime(void)
{
    uint32_t seconds = millis() / 1000;
    uint8_t minutes = (seconds / 60) % 60;

    // if we fly for more than an hour, something's wrong anyway
    sendDataHead(0x17);
    serialize16(minutes << 8);
    sendDataHead(0x18);
    serialize16(seconds % 60);
}

static void sendGPS(void)
{
    sendDataHead(0x13);
    serialize16(abs(GPS_coord[LAT]) / 100000);
    sendDataHead(0x13 + 8);
    serialize16((abs(GPS_coord[LAT]) / 100000) % 10000);

    sendDataHead(0x1B + 8);
    serialize16(GPS_coord[LAT] < 0 ? 'S' : 'N');

    sendDataHead(0x12);
    serialize16(abs(GPS_coord[LON]) / 100000);
    sendDataHead(0x12 + 8);
    serialize16((abs(GPS_coord[LON]) / 100000) % 10000);
    sendDataHead(0x1A + 8);
    serialize16(GPS_coord[LON] < 0 ? 'W' : 'E');
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

        // Frame 1: sent every 200ms
        sendAccel();
        sendBaro();
        sendTemperature1();
        sendTelemetryTail();

        if ((cycleNum % 5) == 0) {      // Frame 2: Sent every 1s
            if (sensors(SENSOR_GPS)) {
                sendGPS();
                sendTelemetryTail();
            }
        }

        if (cycleNum == 25) {     //Frame 3: Sent every 5s
            cycleNum = 0;
            sendTime();
            sendTelemetryTail();
        }
    }
}
