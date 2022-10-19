/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_hott.c
 *
 * Authors:
 * Dominic Clifton - Hydra - Software Serial, Electronics, Hardware Integration and debugging, HoTT Code cleanup and fixes, general telemetry improvements.
 * Carsten Giesen - cGiesen - Baseflight port
 * Oliver Bayer - oBayer - MultiWii-HoTT, HoTT reverse engineering
 * Adam Majerczyk - HoTT-for-ardupilot from which some information and ideas are borrowed.
 * Scavanger & Ziege-One: CMS Textmode addon
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
 * There is a technical discussion (in German) about HoTT here
 * http://www.rc-network.de/forum/showthread.php/281496-Graupner-HoTT-Telemetrie-Sensoren-Eigenbau-DIY-Telemetrie-Protokoll-entschl%C3%BCsselt/page21
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"


#ifdef USE_TELEMETRY_HOTT

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/position.h"
#include "flight/pid.h"

#include "io/gps.h"

#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "sensors/sensors.h"

#include "telemetry/hott.h"
#include "telemetry/telemetry.h"

#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
#include "scheduler/scheduler.h"
#include "io/displayport_hott.h"

#define HOTT_TEXTMODE_TASK_PERIOD 1000
#define HOTT_TEXTMODE_RX_SCHEDULE 5000
#define HOTT_TEXTMODE_TX_DELAY_US 1000
#endif

//#define HOTT_DEBUG

#define HOTT_MESSAGE_PREPARATION_FREQUENCY_5_HZ ((1000 * 1000) / 5)
#define HOTT_RX_SCHEDULE 4000
#define HOTT_TX_DELAY_US 3000
#define MILLISECONDS_IN_A_SECOND 1000

static uint32_t rxSchedule = HOTT_RX_SCHEDULE;
static uint32_t txDelayUs = HOTT_TX_DELAY_US;

static uint32_t lastHoTTRequestCheckAt = 0;
static uint32_t lastMessagesPreparedAt = 0;
static uint32_t lastHottAlarmSoundTime = 0;

static bool hottIsSending = false;

static uint8_t *hottMsg = NULL;
static uint8_t hottMsgRemainingBytesToSendCount;
static uint8_t hottMsgCrc;

#define HOTT_CRC_SIZE (sizeof(hottMsgCrc))

#define HOTT_BAUDRATE 19200
#define HOTT_PORT_MODE MODE_RXTX // must be opened in RXTX so that TX and RX pins are allocated.

static serialPort_t *hottPort = NULL;
static const serialPortConfig_t *portConfig;

static bool hottTelemetryEnabled =  false;
static portSharing_e hottPortSharing;

static HOTT_GPS_MSG_t hottGPSMessage;
static HOTT_EAM_MSG_t hottEAMMessage;

#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
static hottTextModeMsg_t hottTextModeMessage;
static bool textmodeIsAlive = false;
static int32_t telemetryTaskPeriod = 0;

static void initialiseTextmodeMessage(hottTextModeMsg_t *msg)
{
    msg->start = HOTT_TEXTMODE_START;
    msg->esc = HOTT_EAM_SENSOR_TEXT_ID;
    msg->warning = 0;
    msg->stop = HOTT_TEXTMODE_STOP;
}
#endif

static void initialiseEAMMessage(HOTT_EAM_MSG_t *msg, size_t size)
{
    memset(msg, 0, size);
    msg->start_byte = 0x7C;
    msg->eam_sensor_id = HOTT_TELEMETRY_EAM_SENSOR_ID;
    msg->sensor_id = HOTT_EAM_SENSOR_TEXT_ID;
    msg->stop_byte = 0x7D;
}

#ifdef USE_GPS
typedef enum {
    GPS_FIX_CHAR_NONE = '-',
    GPS_FIX_CHAR_2D = '2',
    GPS_FIX_CHAR_3D = '3',
    GPS_FIX_CHAR_DGPS = 'D'
} gpsFixChar_e;

static void initialiseGPSMessage(HOTT_GPS_MSG_t *msg, size_t size)
{
    memset(msg, 0, size);
    msg->start_byte = 0x7C;
    msg->gps_sensor_id = HOTT_TELEMETRY_GPS_SENSOR_ID;
    msg->sensor_id = HOTT_GPS_SENSOR_TEXT_ID;
    msg->stop_byte = 0x7D;
}
#endif

static void initialiseMessages(void)
{
    initialiseEAMMessage(&hottEAMMessage, sizeof(hottEAMMessage));
#ifdef USE_GPS
    initialiseGPSMessage(&hottGPSMessage, sizeof(hottGPSMessage));
#endif
#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
    initialiseTextmodeMessage(&hottTextModeMessage);
#endif
}

#ifdef USE_GPS
void addGPSCoordinates(HOTT_GPS_MSG_t *hottGPSMessage, int32_t latitude, int32_t longitude)
{
    int16_t deg = latitude / GPS_DEGREES_DIVIDER;
    int32_t sec = (latitude - (deg * GPS_DEGREES_DIVIDER)) * 6;
    int8_t min = sec / 1000000L;
    sec = (sec % 1000000L) / 100L;
    uint16_t degMin = (deg * 100L) + min;

    hottGPSMessage->pos_NS = (latitude < 0);
    hottGPSMessage->pos_NS_dm_L = degMin;
    hottGPSMessage->pos_NS_dm_H = degMin >> 8;
    hottGPSMessage->pos_NS_sec_L = sec;
    hottGPSMessage->pos_NS_sec_H = sec >> 8;

    deg = longitude / GPS_DEGREES_DIVIDER;
    sec = (longitude - (deg * GPS_DEGREES_DIVIDER)) * 6;
    min = sec / 1000000L;
    sec = (sec % 1000000L) / 100L;
    degMin = (deg * 100L) + min;

    hottGPSMessage->pos_EW = (longitude < 0);
    hottGPSMessage->pos_EW_dm_L = degMin;
    hottGPSMessage->pos_EW_dm_H = degMin >> 8;
    hottGPSMessage->pos_EW_sec_L = sec;
    hottGPSMessage->pos_EW_sec_H = sec >> 8;
}

void hottPrepareGPSResponse(HOTT_GPS_MSG_t *hottGPSMessage)
{
    hottGPSMessage->gps_satelites = gpsSol.numSat;

    if (!STATE(GPS_FIX)) {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_NONE;
        return;
    }

    if (gpsSol.numSat >= GPS_MIN_SAT_COUNT) {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_3D;
    } else {
        hottGPSMessage->gps_fix_char = GPS_FIX_CHAR_2D;
    }

    addGPSCoordinates(hottGPSMessage, gpsSol.llh.lat, gpsSol.llh.lon);

    // GPS Speed is returned in cm/s (from io/gps.c) and must be sent in km/h (Hott requirement)
    const uint16_t speed = (gpsSol.groundSpeed * 36) / 1000;
    hottGPSMessage->gps_speed_L = speed & 0x00FF;
    hottGPSMessage->gps_speed_H = speed >> 8;

    hottGPSMessage->home_distance_L = GPS_distanceToHome & 0x00FF;
    hottGPSMessage->home_distance_H = GPS_distanceToHome >> 8;

    int32_t altitudeM = getEstimatedAltitudeCm() / 100;

    const uint16_t hottGpsAltitude = constrain(altitudeM + HOTT_GPS_ALTITUDE_OFFSET, 0 , UINT16_MAX); // gpsSol.llh.alt in m ; offset = 500 -> O m

    hottGPSMessage->altitude_L = hottGpsAltitude & 0x00FF;
    hottGPSMessage->altitude_H = hottGpsAltitude >> 8;

    hottGPSMessage->home_direction = GPS_directionToHome / 10;
}
#endif

static bool shouldTriggerBatteryAlarmNow(void)
{
    return ((millis() - lastHottAlarmSoundTime) >= (telemetryConfig()->hottAlarmSoundInterval * MILLISECONDS_IN_A_SECOND));
}

static inline void updateAlarmBatteryStatus(HOTT_EAM_MSG_t *hottEAMMessage)
{
    if (shouldTriggerBatteryAlarmNow()) {
        lastHottAlarmSoundTime = millis();
        const batteryState_e voltageState = getVoltageState();
        const batteryState_e consumptionState = getConsumptionState();
        if (voltageState == BATTERY_WARNING  || voltageState == BATTERY_CRITICAL) {
            hottEAMMessage->warning_beeps = 0x10;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_BATTERY_1;
        } else if (consumptionState == BATTERY_WARNING  || consumptionState == BATTERY_CRITICAL) {
            hottEAMMessage->warning_beeps = 0x16;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_MAH;
        } else {
            hottEAMMessage->warning_beeps = HOTT_EAM_ALARM1_FLAG_NONE;
            hottEAMMessage->alarm_invers1 = HOTT_EAM_ALARM1_FLAG_NONE;
        }
    }
}

static inline void hottEAMUpdateBattery(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const uint16_t volt = getLegacyBatteryVoltage();
    hottEAMMessage->main_voltage_L = volt & 0xFF;
    hottEAMMessage->main_voltage_H = volt >> 8;
    hottEAMMessage->batt1_voltage_L = volt & 0xFF;
    hottEAMMessage->batt1_voltage_H = volt >> 8;

    updateAlarmBatteryStatus(hottEAMMessage);
}

static inline void hottEAMUpdateCurrentMeter(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t amp = getAmperage() / 10;
    hottEAMMessage->current_L = amp & 0xFF;
    hottEAMMessage->current_H = amp >> 8;
}

static inline void hottEAMUpdateBatteryDrawnCapacity(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t mAh = getMAhDrawn() / 10;
    hottEAMMessage->batt_cap_L = mAh & 0xFF;
    hottEAMMessage->batt_cap_H = mAh >> 8;
}

static inline void hottEAMUpdateAltitude(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const uint16_t hottEamAltitude = (getEstimatedAltitudeCm() / 100) + HOTT_EAM_OFFSET_HEIGHT;

    hottEAMMessage->altitude_L = hottEamAltitude & 0x00FF;
    hottEAMMessage->altitude_H = hottEamAltitude >> 8;
}

#ifdef USE_VARIO
static inline void hottEAMUpdateClimbrate(HOTT_EAM_MSG_t *hottEAMMessage)
{
    const int32_t vario = getEstimatedVario();
    hottEAMMessage->climbrate_L = (30000 + vario) & 0x00FF;
    hottEAMMessage->climbrate_H = (30000 + vario) >> 8;
    hottEAMMessage->climbrate3s = 120 + (vario / 100);
}
#endif

void hottPrepareEAMResponse(HOTT_EAM_MSG_t *hottEAMMessage)
{
    // Reset alarms
    hottEAMMessage->warning_beeps = 0x0;
    hottEAMMessage->alarm_invers1 = 0x0;

    hottEAMUpdateBattery(hottEAMMessage);
    hottEAMUpdateCurrentMeter(hottEAMMessage);
    hottEAMUpdateBatteryDrawnCapacity(hottEAMMessage);
    hottEAMUpdateAltitude(hottEAMMessage);
#ifdef USE_VARIO
    hottEAMUpdateClimbrate(hottEAMMessage);
#endif
}

static void hottSerialWrite(uint8_t c)
{
    static uint8_t serialWrites = 0;
    serialWrites++;
    serialWrite(hottPort, c);
}

void freeHoTTTelemetryPort(void)
{
    closeSerialPort(hottPort);
    hottPort = NULL;
    hottTelemetryEnabled = false;
}

void initHoTTTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_HOTT);

    if (!portConfig) {
        return;
    }

    hottPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_HOTT);

#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
    hottDisplayportRegister();
#endif

    initialiseMessages();
}

static void flushHottRxBuffer(void)
{
    while (serialRxBytesWaiting(hottPort) > 0) {
        serialRead(hottPort);
    }
}

static void workAroundForHottTelemetryOnUsart(serialPort_t *instance, portMode_e mode)
{
    closeSerialPort(hottPort);

    portOptions_e portOptions = telemetryConfig()->telemetry_inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED;

    if (telemetryConfig()->halfDuplex) {
        portOptions |= SERIAL_BIDIR;
    }

    hottPort = openSerialPort(instance->identifier, FUNCTION_TELEMETRY_HOTT, NULL, NULL, HOTT_BAUDRATE, mode, portOptions);
}

static bool hottIsUsingHardwareUART(void)
{
    return !(portConfig->identifier == SERIAL_PORT_SOFTSERIAL1 || portConfig->identifier == SERIAL_PORT_SOFTSERIAL2);
}

static void hottConfigurePortForTX(void)
{
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if (hottIsUsingHardwareUART()) {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_TX);
    } else {
        serialSetMode(hottPort, MODE_TX);
    }
    hottIsSending = true;
    hottMsgCrc = 0;
}

static void hottConfigurePortForRX(void)
{
    // FIXME temorary workaround for HoTT not working on Hardware serial ports due to hardware/softserial serial port initialisation differences
    if (hottIsUsingHardwareUART()) {
        workAroundForHottTelemetryOnUsart(hottPort, MODE_RX);
    } else {
        serialSetMode(hottPort, MODE_RX);
    }
    hottMsg = NULL;
    hottIsSending = false;
    flushHottRxBuffer();
}

void configureHoTTTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    portOptions_e portOptions = SERIAL_NOT_INVERTED;

    if (telemetryConfig()->halfDuplex) {
        portOptions |= SERIAL_BIDIR;
    }

    hottPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_HOTT, NULL, NULL, HOTT_BAUDRATE, HOTT_PORT_MODE, portOptions);

    if (!hottPort) {
        return;
    }

    hottConfigurePortForRX();

    hottTelemetryEnabled = true;
}

static void hottSendResponse(uint8_t *buffer, int length)
{
    if (hottIsSending) {
        return;
    }

    hottMsg = buffer;
    hottMsgRemainingBytesToSendCount = length + HOTT_CRC_SIZE;
}

static inline void hottSendGPSResponse(void)
{
    hottSendResponse((uint8_t *)&hottGPSMessage, sizeof(hottGPSMessage));
}

static inline void hottSendEAMResponse(void)
{
    hottSendResponse((uint8_t *)&hottEAMMessage, sizeof(hottEAMMessage));
}

static void hottPrepareMessages(void)
{
    hottPrepareEAMResponse(&hottEAMMessage);
#ifdef USE_GPS
    hottPrepareGPSResponse(&hottGPSMessage);
#endif
}

#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
static void hottTextmodeStart(void)
{
    // Increase menu speed
    taskInfo_t taskInfo;
    getTaskInfo(TASK_TELEMETRY, &taskInfo);
    telemetryTaskPeriod = taskInfo.desiredPeriodUs;
    rescheduleTask(TASK_TELEMETRY, TASK_PERIOD_HZ(HOTT_TEXTMODE_TASK_PERIOD));

    rxSchedule = HOTT_TEXTMODE_RX_SCHEDULE;
    txDelayUs = HOTT_TEXTMODE_TX_DELAY_US;
}

static void hottTextmodeStop(void)
{
    // Set back to avoid slow down of the FC
    if (telemetryTaskPeriod > 0) {
        rescheduleTask(TASK_TELEMETRY, telemetryTaskPeriod);
        telemetryTaskPeriod = 0;
    }

    rxSchedule = HOTT_RX_SCHEDULE;
    txDelayUs = HOTT_TX_DELAY_US;
}

bool hottTextmodeIsAlive(void)
{
    return textmodeIsAlive;
}

void hottTextmodeGrab(void)
{
    hottTextModeMessage.esc = HOTT_EAM_SENSOR_TEXT_ID;
}

void hottTextmodeExit(void)
{
    hottTextModeMessage.esc = HOTT_TEXTMODE_ESC;
}

void hottTextmodeWriteChar(uint8_t column, uint8_t row, char c)
{
    if (column < HOTT_TEXTMODE_DISPLAY_COLUMNS && row < HOTT_TEXTMODE_DISPLAY_ROWS) {
        if (hottTextModeMessage.txt[row][column] != c)
            hottTextModeMessage.txt[row][column] = c;
    }
}

static void processHottTextModeRequest(const uint8_t cmd)
{
    static bool setEscBack = false;

    if (!textmodeIsAlive) {
        hottTextmodeStart();
        textmodeIsAlive = true;
    }

    if ((cmd & 0xF0) != HOTT_EAM_SENSOR_TEXT_ID) {
        return;
    }

    if (setEscBack) {
        hottTextModeMessage.esc = HOTT_EAM_SENSOR_TEXT_ID;
        setEscBack = false;
    }

    if (hottTextModeMessage.esc != HOTT_TEXTMODE_ESC) {
        hottCmsOpen();
    } else {
        setEscBack = true;
    }

    hottSetCmsKey(cmd & 0x0f, hottTextModeMessage.esc == HOTT_TEXTMODE_ESC);
    hottSendResponse((uint8_t *)&hottTextModeMessage, sizeof(hottTextModeMessage));
}
#endif

static void processBinaryModeRequest(uint8_t address)
{
#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
    if (textmodeIsAlive) {
        hottTextmodeStop();
        textmodeIsAlive = false;
    }
#endif

#ifdef HOTT_DEBUG
    static uint8_t hottBinaryRequests = 0;
    static uint8_t hottGPSRequests = 0;
    static uint8_t hottEAMRequests = 0;
#endif

    switch (address) {
#ifdef USE_GPS
    case 0x8A:
#ifdef HOTT_DEBUG
        hottGPSRequests++;
#endif
        if (sensors(SENSOR_GPS)) {
            hottSendGPSResponse();
        }
        break;
#endif
    case 0x8E:
#ifdef HOTT_DEBUG
        hottEAMRequests++;
#endif
        hottSendEAMResponse();
        break;
    }

#ifdef HOTT_DEBUG
    hottBinaryRequests++;
    debug[0] = hottBinaryRequests;
#ifdef USE_GPS
    debug[1] = hottGPSRequests;
#endif
    debug[2] = hottEAMRequests;
#endif
}

static void hottCheckSerialData(uint32_t currentMicros)
{
    static bool lookingForRequest = true;

    const uint8_t bytesWaiting = serialRxBytesWaiting(hottPort);

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
        bool enoughTimePassed = currentMicros - lastHoTTRequestCheckAt >= rxSchedule;

        if (!enoughTimePassed) {
            return;
        }
        lookingForRequest = true;
    }

    const uint8_t requestId = serialRead(hottPort);
    const uint8_t address = serialRead(hottPort);

    if ((requestId == 0) || (requestId == HOTT_BINARY_MODE_REQUEST_ID) || (address == HOTT_TELEMETRY_NO_SENSOR_ID)) {
    /*
     * FIXME the first byte of the HoTT request frame is ONLY either 0x80 (binary mode) or 0x7F (text mode).
     * The binary mode is read as 0x00 (error reading the upper bit) while the text mode is correctly decoded.
     * The (requestId == 0) test is a workaround for detecting the binary mode with no ambiguity as there is only
     * one other valid value (0x7F) for text mode.
     * The error reading for the upper bit should nevertheless be fixed
     */
        processBinaryModeRequest(address);
    }
#if defined (USE_HOTT_TEXTMODE) && defined (USE_CMS)
    else if (requestId == HOTTV4_TEXT_MODE_REQUEST_ID) {
        processHottTextModeRequest(address);
    }
#endif
}

static void hottSendTelemetryData(void)
{

    if (!hottIsSending) {
        hottConfigurePortForTX();
        return;
    }

    if (hottMsgRemainingBytesToSendCount == 0) {
        hottConfigurePortForRX();
        return;
    }

    --hottMsgRemainingBytesToSendCount;
    if (hottMsgRemainingBytesToSendCount == 0) {
        hottSerialWrite(hottMsgCrc++);
        return;
    }

    hottMsgCrc += *hottMsg;
    hottSerialWrite(*hottMsg++);
}

static inline bool shouldPrepareHoTTMessages(uint32_t currentMicros)
{
    return currentMicros - lastMessagesPreparedAt >= HOTT_MESSAGE_PREPARATION_FREQUENCY_5_HZ;
}

static inline bool shouldCheckForHoTTRequest(void)
{
    if (hottIsSending) {
        return false;
    }
    return true;
}

void checkHoTTTelemetryState(void)
{
    const bool newTelemetryEnabledValue = telemetryDetermineEnabledState(hottPortSharing);

    if (newTelemetryEnabledValue == hottTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue) {
        configureHoTTTelemetryPort();
    } else {
        freeHoTTTelemetryPort();
    }
}

void handleHoTTTelemetry(timeUs_t currentTimeUs)
{
    static timeUs_t serialTimer;

    if (!hottTelemetryEnabled) {
        return;
    }

    if (shouldPrepareHoTTMessages(currentTimeUs)) {
        hottPrepareMessages();
        lastMessagesPreparedAt = currentTimeUs;
    }

    if (shouldCheckForHoTTRequest()) {
        hottCheckSerialData(currentTimeUs);
    }

    if (!hottMsg)
        return;

    if (hottIsSending) {
        if (currentTimeUs - serialTimer < txDelayUs) {
            return;
        }
    }
    hottSendTelemetryData();
    serialTimer = currentTimeUs;
}

#endif
