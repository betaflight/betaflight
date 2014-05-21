/*
 * telemetry_hott.c
 *
 * Authors:
 * Dominic Clifton - Hydra - Software Serial, Electronics, Hardware Integration and debugging, HoTT Code cleanup and fixes, general telemetry improvements.
 * Carsten Giesen - cGiesen - Baseflight port
 * Oliver Bayer - oBayer - MultiWii-HoTT, HoTT reverse engineering
 * Adam Majerczyk - HoTT-for-ardupilot from which some information and ideas are borrowed.
 *
 * https://github.com/obayer/MultiWii-HoTT
 * https://github.com/oBayer/MultiHoTT-Module
 * https://code.google.com/p/hott-for-ardupilot
 *
 * HoTT is implemented in Graupner equipment using a bi-directional protocol over a single wire.
 *
 * Generally the receiver sends a single request byte out using normal uart signals, then waits a short period for a
 * multiple byte response and checksum byte before it sends out the next request byte.
 * Each response byte must be send with a protocol specific delay between them.
 *
 * Serial ports use two wires but HoTT uses a single wire so some electronics are required so that
 * the signals don't get mixed up.  When cleanflight transmits it should not receive it's own transmission.
 *
 * Connect as follows:
 * HoTT TX/RX -> Serial RX (connect directly)
 * Serial TX -> 1N4148 Diode -(|  )-> HoTT TX/RX (connect via diode)
 *
 * The diode should be arranged to allow the data signals to flow the right way
 * -(|  )- == Diode, | indicates cathode marker.
 *
 * As noticed by Skrebber the GR-12 (and probably GR-16/24, too) are based on a PIC 24FJ64GA-002, which has 5V tolerant digital pins.
 *
 * Note: The softserial ports are not listed as 5V tolerant in the STM32F103xx data sheet pinouts and pin description
 * section.  Verify if you require a 5v/3.3v level shifters.  The softserial port should not be inverted.
 *
 * There is a technical discussion (in German) about HoTT here
 * http://www.rc-network.de/forum/showthread.php/281496-Graupner-HoTT-Telemetrie-Sensoren-Eigenbau-DIY-Telemetrie-Protokoll-entschl%C3%BCsselt/page21
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"

#include "drivers/system_common.h"

#include "drivers/serial_common.h"
#include "serial_common.h"

#include "runtime_config.h"

#include "sensors_common.h"

#include "flight_common.h"
#include "gps_common.h"
#include "battery.h"

#include "telemetry_common.h"
#include "telemetry_hott.h"

#define HOTT_DEBUG
#define HOTT_DEBUG_SHOW_EAM_UPDATE_TOGGLE
#define HOTT_DEBUG_USE_EAM_TEST_DATA
extern int16_t debug[4];

static serialPort_t *hottPort;

#define HOTT_BAUDRATE 19200
#define HOTT_INITIAL_PORT_MODE MODE_RX

extern telemetryConfig_t *telemetryConfig;


const uint8_t kHoTTv4BinaryPacketSize = 45;
const uint8_t kHoTTv4TextPacketSize = 173;
static HoTTV4GPSModule_t HoTTV4GPSModule;
static HoTTV4ElectricAirModule_t HoTTV4ElectricAirModule;

static void hottV4SerialWrite(uint8_t c);

static void hottV4GPSUpdate(void);
static void hottV4EAMUpdateBattery(void);
static void hottV4EAMUpdateTemperatures(void);
bool batteryWarning;

/*
 * Sends HoTTv4 capable GPS telemetry frame.
 */

void hottV4FormatGPSResponse(void)
{
    memset(&HoTTV4GPSModule, 0, sizeof(HoTTV4GPSModule));

    // Minimum data set for EAM
    HoTTV4GPSModule.startByte = 0x7C;
    HoTTV4GPSModule.sensorID = HOTTV4_GPS_SENSOR_ID;
    HoTTV4GPSModule.sensorTextID = HOTTV4_GPS_SENSOR_TEXT_ID;
    HoTTV4GPSModule.endByte = 0x7D;

    // Reset alarms
    HoTTV4GPSModule.alarmTone = 0x0;
    HoTTV4GPSModule.alarmInverse1 = 0x0;

    hottV4GPSUpdate();
}

void hottV4GPSUpdate(void)
{
    // Number of Satelites
    HoTTV4GPSModule.GPSNumSat = GPS_numSat;
    if (f.GPS_FIX > 0) {
        // GPS fix
        HoTTV4GPSModule.GPS_fix = 0x66; // Displays a 'f' for fix

        // latitude
        HoTTV4GPSModule.LatitudeNS = (GPS_coord[LAT] < 0);
        uint8_t deg = GPS_coord[LAT] / 100000;
        uint32_t sec = (GPS_coord[LAT] - (deg * 100000)) * 6;
        uint8_t min = sec / 10000;
        sec = sec % 10000;
        uint16_t degMin = (deg * 100) + min;
        HoTTV4GPSModule.LatitudeMinLow = degMin;
        HoTTV4GPSModule.LatitudeMinHigh = degMin >> 8;
        HoTTV4GPSModule.LatitudeSecLow = sec;
        HoTTV4GPSModule.LatitudeSecHigh = sec >> 8;

        // longitude
        HoTTV4GPSModule.longitudeEW = (GPS_coord[LON] < 0);
        deg = GPS_coord[LON] / 100000;
        sec = (GPS_coord[LON] - (deg * 100000)) * 6;
        min = sec / 10000;
        sec = sec % 10000;
        degMin = (deg * 100) + min;
        HoTTV4GPSModule.longitudeMinLow = degMin;
        HoTTV4GPSModule.longitudeMinHigh = degMin >> 8;
        HoTTV4GPSModule.longitudeSecLow = sec;
        HoTTV4GPSModule.longitudeSecHigh = sec >> 8;

        // GPS Speed in km/h
        uint16_t speed = (GPS_speed / 100) * 36; // 0.1m/s * 0.36 = km/h
        HoTTV4GPSModule.GPSSpeedLow = speed & 0x00FF;
        HoTTV4GPSModule.GPSSpeedHigh = speed >> 8;

        // Distance to home
        HoTTV4GPSModule.distanceLow = GPS_distanceToHome & 0x00FF;
        HoTTV4GPSModule.distanceHigh = GPS_distanceToHome >> 8;

        // Altitude
        HoTTV4GPSModule.altitudeLow = GPS_altitude & 0x00FF;
        HoTTV4GPSModule.altitudeHigh = GPS_altitude >> 8;

        // Direction to home
        HoTTV4GPSModule.HomeDirection = GPS_directionToHome;
    } else {
        HoTTV4GPSModule.GPS_fix = 0x20; // Displays a ' ' to show nothing or clear the old value
    }
}

/**
 * Writes cell 1-4 high, low values and if not available
 * calculates vbat.
 */

static uint8_t updateCount = 0;

static void hottV4EAMUpdateBattery(void)
{
#ifdef HOTT_DEBUG_USE_EAM_TEST_DATA
    HoTTV4ElectricAirModule.cell1L = 3.30f * 10 * 5; // 2mv step - 3.30v
    HoTTV4ElectricAirModule.cell1H = 4.20f * 10 * 5; // 2mv step - 4.20v

    HoTTV4ElectricAirModule.cell2L = 0;
    HoTTV4ElectricAirModule.cell2H = 0;

    HoTTV4ElectricAirModule.cell3L = 0;
    HoTTV4ElectricAirModule.cell3H = 0;

    HoTTV4ElectricAirModule.cell4L = 0;
    HoTTV4ElectricAirModule.cell4H = 0;
#endif

    HoTTV4ElectricAirModule.driveVoltageLow = vbat & 0xFF;
    HoTTV4ElectricAirModule.driveVoltageHigh = vbat >> 8;
    HoTTV4ElectricAirModule.battery1Low = vbat & 0xFF;
    HoTTV4ElectricAirModule.battery1High = vbat >> 8;

#if 0
    HoTTV4ElectricAirModule.battery2Low = 0 & 0xFF;
    HoTTV4ElectricAirModule.battery2High = 0 >> 8;

    if (batteryWarning) {
        HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationUndervoltage;
        HoTTV4ElectricAirModule.alarmInverse1 |= 0x80; // Invert Voltage display
    }
#endif

#ifdef HOTT_DEBUG_SHOW_EAM_UPDATE_TOGGLE
    if (updateCount & 1) {
        HoTTV4ElectricAirModule.alarmInverse1 |= 0x80; // Invert Voltage display
    }
#endif
}

static void hottV4EAMUpdateTemperatures(void)
{
    HoTTV4ElectricAirModule.temp1 = 20 + 0;
    HoTTV4ElectricAirModule.temp2 = 20;

#if 0
    if (HoTTV4ElectricAirModule.temp1 >= (20 + MultiHoTTModuleSettings.alarmTemp1)) {
        HoTTV4ElectricAirModule.alarmTone = HoTTv4NotificationMaxTemperature;
        HoTTV4ElectricAirModule.alarmInverse |= 0x8; // Invert Temp1 display
    }
#endif
}


/**
 * Sends HoTTv4 capable EAM telemetry frame.
 */
void hottV4FormatEAMResponse(void)
{
    memset(&HoTTV4ElectricAirModule, 0, sizeof(HoTTV4ElectricAirModule));

    // Minimum data set for EAM
    HoTTV4ElectricAirModule.startByte = 0x7C;
    HoTTV4ElectricAirModule.sensorID = HOTTV4_ELECTRICAL_AIR_SENSOR_ID;
    HoTTV4ElectricAirModule.sensorTextID = HOTTV4_ELECTRICAL_AIR_SENSOR_TEXT_ID;
    HoTTV4ElectricAirModule.endByte = 0x7D;

    // Reset alarms
    HoTTV4ElectricAirModule.alarmTone = 0x0;
    HoTTV4ElectricAirModule.alarmInverse1 = 0x0;

    hottV4EAMUpdateBattery();
    hottV4EAMUpdateTemperatures();

    HoTTV4ElectricAirModule.current = 0 / 10;
    HoTTV4ElectricAirModule.height = OFFSET_HEIGHT + 0;
    HoTTV4ElectricAirModule.m2s = OFFSET_M2S;
    HoTTV4ElectricAirModule.m3s = OFFSET_M3S;
}

static void hottV4SerialWrite(uint8_t c)
{
    serialWrite(hottPort, c);
}

static portMode_t previousPortMode;
static uint32_t previousBaudRate;

void freeHoTTTelemetryPort(void)
{
    // FIXME only need to do this if the port is shared
    serialSetMode(hottPort, previousPortMode);
    serialSetBaudRate(hottPort, previousBaudRate);

    endSerialPortFunction(hottPort, FUNCTION_TELEMETRY);
}

void configureHoTTTelemetryPort(telemetryConfig_t *telemetryConfig)
{
    hottPort = findOpenSerialPort(FUNCTION_TELEMETRY);
    if (hottPort) {
        previousPortMode = hottPort->mode;
        previousBaudRate = hottPort->baudRate;

        //waitForSerialPortToFinishTransmitting(hottPort); // FIXME locks up the system

        serialSetBaudRate(hottPort, HOTT_BAUDRATE);
        serialSetMode(hottPort, HOTT_INITIAL_PORT_MODE);
        beginSerialPortFunction(hottPort, FUNCTION_TELEMETRY);
    } else {
        hottPort = openSerialPort(FUNCTION_TELEMETRY, NULL, HOTT_BAUDRATE, HOTT_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

        // FIXME only need to do this if the port is shared
        previousPortMode = hottPort->mode;
        previousBaudRate = hottPort->baudRate;
    }
}


#define CYCLETIME_50_HZ ((1000 * 1000) / 50)
#define HOTT_RX_SCHEDULE 4000
#define HOTTV4_TX_DELAY_US 3000

static uint32_t lastHoTTRequestCheckAt = 0;
static uint32_t lastMessagePreparedAt = 0;

static bool hottIsSending = false;

static uint8_t *hottMsg = NULL;
static uint8_t hottMsgRemainingBytesToSendCount;
static uint8_t hottMsgCrc;

#define HOTT_CRC_SIZE (sizeof(hottMsgCrc))

void hottV4SendResponse(uint8_t *buffer, int length)
{
    if(hottIsSending) {
        return;
    }

    hottMsg = buffer;
    hottMsgRemainingBytesToSendCount = length;// + HOTT_CRC_SIZE;
}

void hottV4SendGPSResponse(void)
{
    hottV4SendResponse((uint8_t *)&HoTTV4GPSModule, sizeof(HoTTV4GPSModule));
}

void hottV4SendEAMResponse(void)
{
    updateCount++;
    hottV4SendResponse((uint8_t *)&HoTTV4ElectricAirModule, sizeof(HoTTV4ElectricAirModule));
}

void hottPrepareMessage(void) {
    hottV4FormatEAMResponse();
    hottV4FormatGPSResponse();
}

void processBinaryModeRequest(uint8_t address) {

    static uint8_t hottBinaryRequests = 0;
    static uint8_t hottGPSRequests = 0;
    static uint8_t hottEAMRequests = 0;


    switch (address) {
        case 0x8A:
            hottGPSRequests++;
            if (sensors(SENSOR_GPS)) {
                hottV4SendGPSResponse();
            }
            break;
        case 0x8E:
            hottEAMRequests++;
            hottV4SendEAMResponse();
            break;
    }

    hottBinaryRequests++;
#ifdef HOTT_DEBUG
    debug[0] = hottBinaryRequests;
    debug[1] = hottGPSRequests;
    debug[2] = hottEAMRequests;
#endif

}

void flushHottRxBuffer(void)
{
    while (serialTotalBytesWaiting(hottPort) > 0) {
        serialRead(hottPort);
    }
}

void hottCheckSerialData(uint32_t currentMicros) {

    static bool lookingForRequest = true;

    uint8_t bytesWaiting = serialTotalBytesWaiting(hottPort);

    if (bytesWaiting <= 1) {
        return;
    }

    if (bytesWaiting != 2) {
        flushHottRxBuffer();
        lookingForRequest = true;
        return;
    }

    if (lookingForRequest) {
        lastHoTTRequestCheckAt = currentMicros;
        lookingForRequest = false;
        return;
    } else {
        bool enoughTimePassed = currentMicros - lastHoTTRequestCheckAt >= HOTT_RX_SCHEDULE;

        if (!enoughTimePassed) {
            return;
        }
        lookingForRequest = true;
    }

    uint8_t requestId = serialRead(hottPort);
    uint8_t address = serialRead(hottPort);

    if (requestId == HOTTV4_BINARY_MODE_REQUEST_ID) {
        processBinaryModeRequest(address);
    }
}

void hottSendTelemetryData(void) {
    if (!hottIsSending) {
        hottIsSending = true;
        serialSetMode(hottPort, MODE_TX);
        hottMsgCrc = 0;
        return;
    }

    if (hottMsgRemainingBytesToSendCount == 0) {
        hottMsg = NULL;
        hottIsSending = false;

        serialSetMode(hottPort, MODE_RX);
        flushHottRxBuffer();
        return;
    }

    --hottMsgRemainingBytesToSendCount;
    if(hottMsgRemainingBytesToSendCount == 0) {
        hottV4SerialWrite(hottMsgCrc++);
        return;
    }

    hottMsgCrc += *hottMsg;
    hottV4SerialWrite(*hottMsg++);
}

bool shouldPrepareHoTTMessage(uint32_t currentMicros)
{
    return currentMicros - lastMessagePreparedAt >= CYCLETIME_50_HZ;
}

bool shouldCheckForHoTTRequest()
{
    if (hottIsSending) {
        return false;
    }
    return true;
}

void handleHoTTTelemetry(void)
{
    static uint32_t serialTimer;
    uint32_t now = micros();


    if (shouldPrepareHoTTMessage(now)) {
        hottPrepareMessage();
        lastMessagePreparedAt = now;
    }

    if (shouldCheckForHoTTRequest()) {
        hottCheckSerialData(now);
    }

    if(!hottMsg)
        return;

    if(hottIsSending) {
        if(now - serialTimer < HOTTV4_TX_DELAY_US) {
            return;
        }
    }
    hottSendTelemetryData();
    serialTimer = now;
}

uint32_t getHoTTTelemetryProviderBaudRate(void) {
    return HOTT_BAUDRATE;
}


