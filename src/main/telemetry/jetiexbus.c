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
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIAL_RX
#ifdef USE_TELEMETRY

#include "build/build_config.h"
#include "build/debug.h"
#include "fc/runtime_config.h"

#include "common/utils.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/time.h"

#include "flight/altitude.h"
#include "flight/imu.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/jetiexbus.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"

#include "telemetry/jetiexbus.h"
#include "telemetry/telemetry.h"

#define EXTEL_DATA_MSG      (0x40)
#define EXTEL_UNMASK_TYPE   (0x3F)
#define EXTEL_SYNC_LEN      1
#define EXTEL_CRC_LEN       1
#define EXTEL_HEADER_LEN    6
#define EXTEL_MAX_LEN       29
#define EXTEL_OVERHEAD      (EXTEL_SYNC_LEN + EXTEL_HEADER_LEN + EXTEL_CRC_LEN)
#define EXTEL_MAX_PAYLOAD   (EXTEL_MAX_LEN - EXTEL_OVERHEAD)
#define EXBUS_MAX_REQUEST_BUFFER_SIZE   (EXBUS_OVERHEAD + EXTEL_MAX_LEN)


enum exTelHeader_e {
    EXTEL_HEADER_SYNC = 0,
    EXTEL_HEADER_TYPE_LEN,
    EXTEL_HEADER_USN_LB,
    EXTEL_HEADER_USN_HB,
    EXTEL_HEADER_LSN_LB,
    EXTEL_HEADER_LSN_HB,
    EXTEL_HEADER_RES,
    EXTEL_HEADER_ID,
    EXTEL_HEADER_DATA
};

enum {
    EXBUS_TRANS_ZERO = 0,
    EXBUS_TRANS_RX_READY,
    EXBUS_TRANS_RX,
    EXBUS_TRANS_IS_TX_COMPLETED,
    EXBUS_TRANS_TX
};

enum exDataType_e {
    EX_TYPE_NONE = 0,
    EX_TYPE_6b   = 0,                // int6_t  Data type 6b (-31 ¸31)
    EX_TYPE_14b  = 1,                // int14_t Data type 14b (-8191 ¸8191)
    EX_TYPE_22b  = 4,                // int22_t Data type 22b (-2097151 ¸2097151)
    EX_TYPE_DT   = 5,                // int22_t Special data type – time and date
    EX_TYPE_30b  = 8,                // int30_t Data type 30b (-536870911 ¸536870911)
    EX_TYPE_GPS  = 9,                // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree.
    EX_TYPE_SENSOR_DISABLED = 240,
    EX_TYPE_SENSOR_ENABLED = 250,
    EX_TYPE_DES  = 255               // only for devicedescription
};

const uint8_t exDataTypeLen[] = {
    [EX_TYPE_6b]  = 1,
    [EX_TYPE_14b] = 2,
    [EX_TYPE_22b] = 3,
    [EX_TYPE_DT]  = 3,
    [EX_TYPE_30b] = 4,
    [EX_TYPE_GPS] = 4
};

typedef struct exBusSensor_s {
    const char *label;
    const char *unit;
    const uint8_t exDataType;
    const uint8_t decimals;
    uint8_t parameter;
} exBusSensor_t;

#define DECIMAL_MASK(decimals) (decimals << 5)

// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "BF D2")
exBusSensor_t jetiExSensors[] = {
    { "BF D1",       "",      EX_TYPE_DES,   EX_TYPE_NONE,    EX_TYPE_SENSOR_ENABLED},     // device descripton
    { "Voltage",     "V",     EX_TYPE_14b,   DECIMAL_MASK(1), EX_TYPE_SENSOR_DISABLED},
    { "Current",     "A",     EX_TYPE_14b,   DECIMAL_MASK(2), EX_TYPE_SENSOR_DISABLED},
    { "Altitude",    "m",     EX_TYPE_14b,   DECIMAL_MASK(2), EX_TYPE_SENSOR_DISABLED},
    { "Capacity",    "mAh",   EX_TYPE_22b,   DECIMAL_MASK(0), EX_TYPE_SENSOR_DISABLED},
    { "Power",       "W",     EX_TYPE_22b,   DECIMAL_MASK(1), EX_TYPE_SENSOR_DISABLED},
    { "Roll angle",  "\xB0",  EX_TYPE_14b,   DECIMAL_MASK(1), EX_TYPE_SENSOR_DISABLED},
    { "Pitch angle", "\xB0",  EX_TYPE_14b,   DECIMAL_MASK(1), EX_TYPE_SENSOR_DISABLED},
    { "Heading",     "\xB0",  EX_TYPE_14b,   DECIMAL_MASK(1), EX_TYPE_SENSOR_DISABLED}
};

// after every 15 sensors increment the step by 2 (e.g. ...EX_VAL15, EX_VAL16 = 17) to skip the device description
enum exSensors_e {
    EX_VOLTAGE = 1,
    EX_CURRENT,
    EX_ALTITUDE,
    EX_CAPACITY,
    EX_POWER,
    EX_ROLL_ANGLE,
    EX_PITCH_ANGLE,
    EX_HEADING
};

#define JETI_EX_SENSOR_COUNT (ARRAYLEN(jetiExSensors))

static uint8_t jetiExBusTelemetryFrame[40];
static uint8_t jetiExBusTransceiveState = EXBUS_TRANS_RX;
static uint8_t activeSensorList[JETI_EX_SENSOR_COUNT];
static uint8_t activeSensors;

static uint8_t sendJetiExBusTelemetry(uint8_t packetID, uint8_t item);

// Jeti Ex Telemetry CRC calculations for a frame
uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen)
{
    uint8_t crc=0;

    for (uint8_t mlen = 0; mlen < msgLen; mlen++) {
        crc  ^= pt[mlen];
        crc = crc ^ (crc << 1) ^ (crc << 2) ^ (0x0e090700 >> ((crc >> 3) & 0x18));
    }
    return(crc);
}

/*
* -----------------------------------------------
*  Jeti Ex Bus Telemetry
* -----------------------------------------------
*/
void initJetiExBusTelemetry(void)
{
    // Init Ex Bus Frame header
    jetiExBusTelemetryFrame[EXBUS_HEADER_SYNC] = 0x3B;       // Startbytes
    jetiExBusTelemetryFrame[EXBUS_HEADER_REQ] = 0x01;
    jetiExBusTelemetryFrame[EXBUS_HEADER_DATA_ID] = 0x3A;    // Ex Telemetry

    // Init Ex Telemetry header
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];
    jetiExTelemetryFrame[EXTEL_HEADER_SYNC] = 0x9F;              // Startbyte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_LB] = 0x1E;            // Serial Number 4 Byte
    jetiExTelemetryFrame[EXTEL_HEADER_USN_HB] = 0xA4;
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_LB] = 0x00;            // increment by telemetry count (%16) > only 15 values per device possible
    jetiExTelemetryFrame[EXTEL_HEADER_LSN_HB] = 0x00;
    jetiExTelemetryFrame[EXTEL_HEADER_RES] = 0x00;               // reserved, by default 0x00

    // Check which sensors are available
    if (batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE) {
        jetiExSensors[EX_VOLTAGE].parameter = EX_TYPE_SENSOR_ENABLED;
    }
    if (batteryConfig()->currentMeterSource != CURRENT_METER_NONE) {
        jetiExSensors[EX_CURRENT].parameter = EX_TYPE_SENSOR_ENABLED;
    }
    if ((batteryConfig()->voltageMeterSource != VOLTAGE_METER_NONE) && (batteryConfig()->currentMeterSource != CURRENT_METER_NONE)) {
        jetiExSensors[EX_POWER].parameter = EX_TYPE_SENSOR_ENABLED;
        jetiExSensors[EX_CAPACITY].parameter = EX_TYPE_SENSOR_ENABLED;
    }
    if (sensors(SENSOR_BARO)) {
        jetiExSensors[EX_ALTITUDE].parameter = EX_TYPE_SENSOR_ENABLED;
    }
    if (sensors(SENSOR_ACC)) {
        jetiExSensors[EX_ROLL_ANGLE].parameter = EX_TYPE_SENSOR_ENABLED;
        jetiExSensors[EX_PITCH_ANGLE].parameter = EX_TYPE_SENSOR_ENABLED;
    }
    if (sensors(SENSOR_MAG)) {
        jetiExSensors[EX_HEADING].parameter = EX_TYPE_SENSOR_ENABLED;
    }

    for (uint8_t item = 0; item < JETI_EX_SENSOR_COUNT; item++ ) {
        if (jetiExSensors[item].parameter == EX_TYPE_SENSOR_ENABLED) {
            activeSensorList[activeSensors] = item;
            activeSensors++;
        }
    }
}

void createExTelemetryTextMessage(uint8_t *exMessage, uint8_t messageID, const exBusSensor_t *sensor)
{
    uint8_t labelLength = strlen(sensor->label);
    uint8_t unitLength = strlen(sensor->unit);

    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_OVERHEAD + labelLength + unitLength;
    exMessage[EXTEL_HEADER_LSN_LB] = messageID & 0xF0;                              // Device ID
    exMessage[EXTEL_HEADER_ID] = messageID & 0x0F;                                  // Sensor ID (%16)
    exMessage[EXTEL_HEADER_DATA] = (labelLength << 3) + unitLength;

    memcpy(&exMessage[EXTEL_HEADER_DATA + 1], sensor->label, labelLength);
    memcpy(&exMessage[EXTEL_HEADER_DATA + 1 + labelLength], sensor->unit, unitLength);

    exMessage[exMessage[EXTEL_HEADER_TYPE_LEN] + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], exMessage[EXTEL_HEADER_TYPE_LEN]);
}

int32_t getSensorValue(uint8_t sensor)
{
    switch(sensor) {
    case EX_VOLTAGE:
        return (getBatteryVoltageLatest());
        break;

    case EX_CURRENT:
        return (getAmperageLatest());
        break;

    case EX_ALTITUDE:
        return (getEstimatedAltitude());
        break;

    case EX_CAPACITY:
        return (getMAhDrawn());
        break;

    case EX_POWER:
        return (getBatteryVoltageLatest() * getAmperageLatest()/100);
        break;

    case EX_ROLL_ANGLE:
        return (attitude.values.roll);
        break;

    case EX_PITCH_ANGLE:
        return (attitude.values.pitch);
        break;

    case EX_HEADING:
        return (attitude.values.yaw);
        break;

    default:
        return -1;
    }
}

uint8_t createExTelemetryValueMessage(uint8_t *exMessage, uint8_t item)
{
    uint8_t sensorItem = activeSensorList[item];
    uint8_t sensorItemMaxGroup = sensorItem | 0xF0;
    uint8_t iCount;
    uint8_t messageSize;
    uint32_t sensorValue;


    exMessage[EXTEL_HEADER_LSN_LB] = item & 0xF0;                                   // Device ID
    uint8_t *p = &exMessage[EXTEL_HEADER_ID];

    while (sensorItem <= sensorItemMaxGroup) {
        *p++ = ((sensorItem & 0x0F) << 4) | jetiExSensors[sensorItem].exDataType;   // Sensor ID (%16) | EX Data Type

        sensorValue = getSensorValue(sensorItem);
        iCount = exDataTypeLen[jetiExSensors[sensorItem].exDataType];
        while (iCount > 1) {
            *p++ = sensorValue;
            sensorValue = sensorValue >> 8;
            iCount--;
        }
        *p++ = (sensorValue & 0x9F) | jetiExSensors[sensorItem].decimals;

        item++;

        if (item > activeSensors) {
            item = 0;
            break;
        }

        sensorItem = activeSensorList[item];
        if (EXTEL_MAX_PAYLOAD <= ((p-&exMessage[EXTEL_HEADER_ID]) + exDataTypeLen[jetiExSensors[sensorItem].exDataType]) + 1) break;
    }

    messageSize = (EXTEL_HEADER_LEN + (p-&exMessage[EXTEL_HEADER_ID]));
    exMessage[EXTEL_HEADER_TYPE_LEN] = EXTEL_DATA_MSG | messageSize;
    exMessage[messageSize + EXTEL_CRC_LEN] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], messageSize);

    return item;        // return the next item
}

void createExBusMessage(uint8_t *exBusMessage, uint8_t *exMessage, uint8_t packetID)
{
    uint16_t crc16;

    exBusMessage[EXBUS_HEADER_PACKET_ID] = packetID;
    exBusMessage[EXBUS_HEADER_SUBLEN] = (exMessage[EXTEL_HEADER_TYPE_LEN] & EXTEL_UNMASK_TYPE) + 2;    // +2: startbyte & CRC8
    exBusMessage[EXBUS_HEADER_MSG_LEN] = EXBUS_OVERHEAD + exBusMessage[EXBUS_HEADER_SUBLEN];

    crc16 = jetiExBusCalcCRC16(exBusMessage, exBusMessage[EXBUS_HEADER_MSG_LEN] - EXBUS_CRC_LEN);
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 2] = crc16;
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 1] = crc16 >> 8;
}

void checkJetiExBusTelemetryState(void)
{
    return;
}

void handleJetiExBusTelemetry(void)
{
    static uint16_t framesLost = 0; // only for debug
    static uint8_t item = 0;
    uint32_t timeDiff;
    // Check if we shall reset frame position due to time

    if (jetiExBusRequestState == EXBUS_STATE_RECEIVED) {

        // to prevent timing issues from request to answer - max. 4ms
        timeDiff = micros() - jetiTimeStampRequest;

        if (timeDiff > 3000) {   // include reserved time
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            framesLost++;
            return;
        }

        if ((jetiExBusRequestFrame[EXBUS_HEADER_DATA_ID] == EXBUS_EX_REQUEST) && (jetiExBusCalcCRC16(jetiExBusRequestFrame, jetiExBusRequestFrame[EXBUS_HEADER_MSG_LEN]) == 0)) {
            // switch to TX mode
            if (serialRxBytesWaiting(jetiExBusPort) == 0) {
                serialSetMode(jetiExBusPort, MODE_TX);
                jetiExBusTransceiveState = EXBUS_TRANS_TX;
                item = sendJetiExBusTelemetry(jetiExBusRequestFrame[EXBUS_HEADER_PACKET_ID], item);
                jetiExBusRequestState = EXBUS_STATE_PROCESSED;
                return;
            }
        } else {
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            return;
        }
    }

    // check the state if transmit is ready
    if (jetiExBusTransceiveState == EXBUS_TRANS_IS_TX_COMPLETED) {
        if (isSerialTransmitBufferEmpty(jetiExBusPort)) {
            serialSetMode(jetiExBusPort, MODE_RX);
            jetiExBusTransceiveState = EXBUS_TRANS_RX;
            jetiExBusRequestState = EXBUS_STATE_ZERO;
        }
    }
}

uint8_t sendJetiExBusTelemetry(uint8_t packetID, uint8_t item)
{
    static uint8_t sensorDescriptionCounter = 0xFF;
    static uint8_t requestLoop = 0;
    uint8_t *jetiExTelemetryFrame = &jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];

    if (requestLoop < 0xFF) {
        while( ++sensorDescriptionCounter < JETI_EX_SENSOR_COUNT) {
            if (jetiExSensors[sensorDescriptionCounter].parameter >= EX_TYPE_SENSOR_ENABLED) {
                break;
            }
        }
        if (sensorDescriptionCounter == JETI_EX_SENSOR_COUNT ) {
            sensorDescriptionCounter = 0;
        }

        createExTelemetryTextMessage(jetiExTelemetryFrame, sensorDescriptionCounter, &jetiExSensors[sensorDescriptionCounter]);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);
        requestLoop++;
    } else {
        if(item >= activeSensors) {
            item = 0;
        }
        item = createExTelemetryValueMessage(jetiExTelemetryFrame, item);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetryFrame, packetID);
    }

    serialWriteBuf(jetiExBusPort, jetiExBusTelemetryFrame, jetiExBusTelemetryFrame[EXBUS_HEADER_MSG_LEN]);
    jetiExBusTransceiveState = EXBUS_TRANS_IS_TX_COMPLETED;

    return item;
}

#endif // TELEMETRY
#endif // SERIAL_RX

