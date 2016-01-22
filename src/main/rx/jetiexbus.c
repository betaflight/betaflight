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
 * Authors:
 * Thomas Miric - marv-t
 *
 * Jeti EX Bus Communication Protocol:
 * http://www.jetimodel.com/en/show-file/642/
 *
 * JETI Telemetry Communication Protocol:
 * http://www.jetimodel.com/en/show-file/26/
 *
 * Following restrictions:
 * Communication speed: 125000 bps
 * Number of channels: 16
 *
 * Connect as follows:
 * Jeti EX Bus -> Serial RX (connect directly)
 * Serial TX -> Resistor(2k4) ->Serial RX
 * In jeti pdf it is different, but if the resistor breaks, the receiver continues to operate.
 *
 */

#include <stdbool.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "io/serial.h"
#include "rx/jetiexbus.h"


#ifdef TELEMETRY

#include <string.h>
#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"
#include "telemetry/telemetry.h"

#endif //TELEMETRY

#include "rx/rx.h"

//
// Serial driver for Jeti EX Bus receiver
//
#define JETIEXBUS_BAUDRATE 125000                           // EX Bus 125000; EX Bus HS 250000 not supported
#define JETIEXBUS_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED )

#define JETIEXBUS_MIN_FRAME_GAP 1                       

#define JETIEXBUS_CHANNEL_COUNT 16                          // most Jeti TX transmit 16 channels
#define JETIEXBUS_MIN_FRAME_SIZE 8
#define JETIEXBUS_HEADER_LEN 6
#define JETIEXBUS_CRC_LEN 2

#define JETIEXBUS_MAX_FRAME_SIZE (JETIEXBUS_HEADER_LEN + JETIEXBUS_CHANNEL_COUNT*2 + JETIEXBUS_CRC_LEN)
#define JETIEXBUS_FRAME_OFFSET (JETIEXBUS_MAX_FRAME_SIZE)
#define JETIEXBUS_MAX_REQUESTFRAME_SIZE 8

#define JETIEXBUS_START_CHANNEL_FRAME        (0x3E)
#define JETIEXBUS_START_REQUEST_FRAME        (0x3D)

#define EXBUS_CHANNELDATA (0x3E03)                          // Frame contains Channel Data
#define EXBUS_CHANNELDATA_TELEMETRY_REQUEST (0x3E01)        // Frame contains Channel Data, but with a request for data


enum {
    EXBUS_STATE_ZERO = 0,
    EXBUS_STATE_IN_PROGRESS,
    EXBUS_STATE_RECEIVED,
    EXBUS_STATE_PROCESSED
};

enum {
    EXBUS_TRANS_ZERO = 0,
    EXBUS_TRANS_RX_READY,
    EXBUS_TRANS_RX,
    EXBUS_TRANS_TX_READY,
    EXBUS_TRANS_TX
};

enum exBusHeader_e {
    EXBUS_HEADER_SYNC = 0,
    EXBUS_HEADER_REQ,
    EXBUS_HEADER_MSG_LEN,
    EXBUS_HEADER_PACKET_ID,
    EXBUS_HEADER_DATA_ID,
    EXBUS_HEADER_SUBLEN,
    EXBUS_HEADER_DATA
};

#ifdef TELEMETRY

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

enum exDataType_e {
    EX_TYPE_6b   = 0, // int6_t  Data type 6b (-31 ¸31)
    EX_TYPE_14b  = 1, // int14_t Data type 14b (-8191 ¸8191)
    EX_TYPE_22b  = 4, // int22_t Data type 22b (-2097151 ¸2097151)
    EX_TYPE_DT   = 5, // int22_t Special data type – time and date
    EX_TYPE_30b  = 8, // int30_t Data type 30b (-536870911 ¸536870911)
    EX_TYPE_GPS  = 9  // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree.
};

typedef struct exBusSensor_s{
    const char *name;
    const char *unit;
    int32_t value;
    const uint8_t exDataType;
    const uint32_t decimals;
} exBusSensor_t;


#define SETMASK(decimals, bytes) (decimals << ((bytes*8)-3))
#define RESETMASK(bytes) (~(3 << ((bytes*8)-3)))

const uint32_t resetMask[]={ RESETMASK(1), RESETMASK(2), RESETMASK(3), RESETMASK(4) };

// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "CF-Dev 1.12 S2")

exBusSensor_t jetiExSensors[] = {
    { "CF-Dev 1.12 S1", "",     0,      0,              0 },
    { "Voltage",        "V",    0,      EX_TYPE_14b,    SETMASK(1,2) },
    { "Current",        "A",    0,      EX_TYPE_14b,    SETMASK(2,2) },
    { "Altitude",       "m",    0,      EX_TYPE_14b,    SETMASK(1,2) },
    { "Capacity",       "mAh",  0,      EX_TYPE_14b,    SETMASK(0,2) }
};

enum exSensors_e {
    EX_VOLTAGE = 1,
    EX_CURRENT,
    EX_ALTITUDE,
    EX_CAPACITY
};

#define JETI_EX_SENSOR_COUNT (sizeof(jetiExSensors) / sizeof(exBusSensor_t))

#endif //TELEMETRY

static serialPort_t *jetiExBusPort;
static serialPortConfig_t *portConfig;

static uint8_t jetiExBusFrameState = EXBUS_STATE_ZERO;
static uint8_t jetiExBusRequestState = EXBUS_STATE_ZERO;
static uint8_t jetiExBusFramePosition;
static uint8_t jetiExBusFrameLength;

// Use max values for ram areas
static uint8_t jetiExBusSerialFrame[JETIEXBUS_MAX_FRAME_SIZE+JETIEXBUS_MAX_REQUESTFRAME_SIZE];
static uint8_t *jetiExBusFrame;


static uint16_t jetiExBusChannelData[JETIEXBUS_CHANNEL_COUNT];
static void jetiExBusDataReceive(uint16_t c);
static uint16_t jetiExBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan);


#ifdef TELEMETRY

static uint8_t jetiExBusTelemetryFrame[JETIEXBUS_MAX_FRAME_SIZE];
static uint8_t jetiExBusTransceiveState = EXBUS_TRANS_RX;
static uint8_t *jetiExTelemetry=&jetiExBusTelemetryFrame[EXBUS_HEADER_DATA];
static void sendJetiExBusTelemetry(uint8_t packetID);

uint8_t updateCRC8( uint8_t crc, uint8_t data );
uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen);

#endif //TELEMETRY

uint16_t updateCRC16( uint16_t crc, uint8_t data );
uint16_t calcCRC16(uint8_t *pt, uint8_t msgLen);


bool jetiExBusInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    
    if (callback) {
        *callback = jetiExBusReadRawRC;
    }

    rxRuntimeConfig->channelCount = JETIEXBUS_CHANNEL_COUNT;
    jetiExBusFramePosition = 0;
    jetiExBusFrameLength = JETIEXBUS_MAX_FRAME_SIZE;

    portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    
    if (!portConfig) {
        return false;
    }
    
    jetiExBusPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, jetiExBusDataReceive, JETIEXBUS_BAUDRATE, MODE_RXTX, JETIEXBUS_OPTIONS );
    serialSetMode(jetiExBusPort, MODE_RX);
    return jetiExBusPort != NULL;
}


// Jeti Ex Bus CRC calculations for single byte
uint16_t updateCRC16( uint16_t crc, uint8_t data )
{
    uint16_t ret_val;
    data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
    data ^= data << 4;
    ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
               ^ (uint8_t)(data >> 4)
               ^ ((uint16_t)data << 3));
    return ret_val;
} 


// Jeti Ex Bus CRC calculations for a frame
uint16_t calcCRC16(uint8_t *pt, uint8_t msgLen)
{
    uint16_t crc16_data = 0;
    uint8_t mlen = 0;
    while(mlen < msgLen) {
        crc16_data = updateCRC16(crc16_data, pt[mlen]);
        mlen++;
    }
    return(crc16_data);
}

#ifdef TELEMETRY

// Jeti Ex Telemetry CRC calculations for single byte
uint8_t updateCRC8( uint8_t crc, uint8_t data )
{
    crc ^= data;
    return (crc ^ (crc << 1) ^ (crc << 2) ^ (0x0e090700 >> ((crc >> 3) & 0x18)));
} 

// Jeti Ex Telemetry CRC calculations for a frame
uint8_t calcCRC8(uint8_t *pt, uint8_t msgLen)
{
    uint8_t crc8_data = 0;
    uint8_t mlen = 0;
    for (mlen = 0; mlen < msgLen; mlen++) {
        crc8_data = updateCRC8(crc8_data, pt[mlen]);
    }
    return(crc8_data);
}

#endif //TELEMETRY


void jetiExBusDecodeFrame(uint8_t *pFrame)
{
    uint16_t value;
    uint8_t frameAddr;

    // Decode header
    switch (((uint16_t)jetiExBusFrame[EXBUS_HEADER_SYNC] << 8) + ((uint16_t)jetiExBusFrame[EXBUS_HEADER_REQ])){

    case EXBUS_CHANNELDATA_TELEMETRY_REQUEST:                   // not yet specified
    case EXBUS_CHANNELDATA:
        for (uint8_t i = 0; i < JETIEXBUS_CHANNEL_COUNT; i++) {
            frameAddr = JETIEXBUS_HEADER_LEN + i * 2;
            value = ((uint16_t)pFrame[frameAddr + 1]) << 8;
            value += (uint16_t)pFrame[frameAddr];
            // Convert to internal format
            jetiExBusChannelData[i] = value >> 3;
        }
        break;
    }
}

void jetiExBusFrameReset(void)
{
    jetiExBusFramePosition = 0;
    jetiExBusFrameLength = JETIEXBUS_MAX_FRAME_SIZE;
}


/*
  supported:
  0x3E 0x01 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data with telemetry request (2nd byte 0x01)
  0x3E 0x03 LEN Packet_ID 0x31 SUB_LEN Data_array CRC16      // Channel Data forbids answering (2nd byte 0x03)
  0x3D 0x01 0x08 Packet_ID 0x3A 0x00 CRC16                   // Telemetry Request EX telemetry (5th byte 0x3A)
  
  other messages - not supported:
  0x3D 0x01 0x09 Packet_ID 0x3B 0x01 0xF0 CRC16              // Jetibox request (5th byte 0x3B)
  ...
  
*/
// Receive ISR callback
static void jetiExBusDataReceive(uint16_t c)
{
    uint32_t now;
    static uint32_t jetiExBusTimeLast = 0;
    static uint32_t jetiExBusTimeInterval;
    static uint16_t crc = 0;
    
    // Check if we shall reset frame position due to time
    now = millis();

    jetiExBusTimeInterval = now - jetiExBusTimeLast;
    jetiExBusTimeLast = now;

    if (jetiExBusTimeInterval > JETIEXBUS_MIN_FRAME_GAP) {
        jetiExBusFrameReset();
        jetiExBusFrameState = EXBUS_STATE_ZERO;
        jetiExBusRequestState = EXBUS_STATE_ZERO;
    }
    
    // Check if we shall start a frame?
    if (jetiExBusFramePosition == 0) {
        switch(c){
        case JETIEXBUS_START_CHANNEL_FRAME:
            jetiExBusFrameState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = &jetiExBusSerialFrame[0];
            break;
            
        case JETIEXBUS_START_REQUEST_FRAME:
            jetiExBusRequestState = EXBUS_STATE_IN_PROGRESS;
            jetiExBusFrame = &jetiExBusSerialFrame[JETIEXBUS_MAX_FRAME_SIZE];
            break;

        default:
            return;
        }
        crc = 0;
    }

    // Store in frame copy
    jetiExBusFrame[jetiExBusFramePosition] = (uint8_t)c;
    crc = updateCRC16(crc, (uint8_t)c);
    jetiExBusFramePosition++;
    
    // Check the header for the message length
    if (jetiExBusFramePosition==JETIEXBUS_HEADER_LEN) {
        if ((jetiExBusFrame[EXBUS_HEADER_MSG_LEN]<=JETIEXBUS_MAX_FRAME_SIZE) && (jetiExBusFrame[EXBUS_HEADER_MSG_LEN]>=JETIEXBUS_MIN_FRAME_SIZE)) {
            jetiExBusFrameLength = jetiExBusFrame[EXBUS_HEADER_MSG_LEN];
        } else {
            jetiExBusFrameReset();                  // not a valid frame
            jetiExBusFrameState = EXBUS_STATE_ZERO;
            jetiExBusRequestState = EXBUS_STATE_ZERO;
            return;
        }
    }

    // Done?
    if (jetiExBusFrameLength == jetiExBusFramePosition) {
        if (crc == 0){
            if (jetiExBusFrameState == EXBUS_STATE_IN_PROGRESS) jetiExBusFrameState = EXBUS_STATE_RECEIVED;
            if (jetiExBusRequestState == EXBUS_STATE_IN_PROGRESS) jetiExBusRequestState =  EXBUS_STATE_RECEIVED;
        }
        jetiExBusFrameReset();
    }
}


// Check if it is time to read a frame from the data...
uint8_t jetiExBusFrameStatus(void)
{
    if (jetiExBusFrameState == EXBUS_STATE_RECEIVED) {
        jetiExBusDecodeFrame(&jetiExBusSerialFrame[0]);
        jetiExBusFrameState = EXBUS_STATE_ZERO;
    } else {
        return SERIAL_RX_FRAME_PENDING;
    }
    return SERIAL_RX_FRAME_COMPLETE;
}


static uint16_t jetiExBusReadRawRC(rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    if (chan >= rxRuntimeConfig->channelCount)
        return 0;

    return (jetiExBusChannelData[chan]);
}

#ifdef TELEMETRY
/*
  -----------------------------------------------
   Jeti Ex Bus Telemetry
  -----------------------------------------------
*/
void createExTelemetrieTextMessage(uint8_t *exMessage, uint8_t messageID, const exBusSensor_t *Sensor)
{
    uint8_t labelLength = strlen(Sensor->name);
    uint8_t unitLength = strlen(Sensor->unit);

    jetiExTelemetry[EXTEL_HEADER_LSN_LB] = messageID & 0xF0;

    exMessage[EXTEL_HEADER_TYPE_LEN] = 8 + labelLength + unitLength;       // 7 header, 1 CRC, label & unit
    exMessage[EXTEL_HEADER_ID] = messageID & 0x0F;                     // ExMessage ID
    exMessage[EXTEL_HEADER_DATA] = (labelLength << 3) + unitLength;

    memcpy(&exMessage[EXTEL_HEADER_DATA + 1], Sensor->name, labelLength);
    memcpy(&exMessage[EXTEL_HEADER_DATA + 1 + labelLength], Sensor->unit, unitLength);

    exMessage[exMessage[EXTEL_HEADER_TYPE_LEN] + 1] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], exMessage[EXTEL_HEADER_TYPE_LEN]);
}


uint8_t createExTelemetrieValueMessage(uint8_t *exMessage, uint8_t itemStart)
{
    uint8_t lastItem;
    uint8_t byteCount = 0;
    uint8_t valueByteCount = 0;
    uint8_t iCount = 0;
    uint32_t sensorValue;


    if ((itemStart % 16) == 0)
        itemStart++;

    lastItem = itemStart + 5;

    if (lastItem > JETI_EX_SENSOR_COUNT)
        lastItem = JETI_EX_SENSOR_COUNT;

    jetiExTelemetry[EXTEL_HEADER_LSN_LB] = itemStart & 0xF0;

    if (itemStart>JETI_EX_SENSOR_COUNT)
        return 0;

    for (uint8_t item=itemStart; item < lastItem; item++){

        switch(jetiExSensors[item].exDataType) {

        case EX_TYPE_6b:
            byteCount = 1;
            break;
        case EX_TYPE_14b:
            byteCount = 2;
            break;
        case EX_TYPE_22b:
        case EX_TYPE_DT:
            byteCount = 3;
            break;
        case EX_TYPE_30b:
        case EX_TYPE_GPS:
            byteCount = 4;
            break;
        }

        exMessage[EXTEL_HEADER_ID + valueByteCount] = ((item & 0xF) << 4) + jetiExSensors[item].exDataType;


        sensorValue = jetiExSensors[item].value;
        if (jetiExSensors[item].value < 0)
            sensorValue &= resetMask[byteCount - 1];

        sensorValue |= jetiExSensors[item].decimals;

        iCount = 0;
        do {
            exMessage[EXTEL_HEADER_DATA + valueByteCount + iCount] = sensorValue;
            sensorValue = sensorValue>>8;
            iCount++;
        } while(iCount < byteCount);

        valueByteCount += 1 + byteCount;
    }

    exMessage[EXTEL_HEADER_TYPE_LEN] = 0x46+valueByteCount;
    exMessage[(exMessage[EXTEL_HEADER_TYPE_LEN] & 0x3F) + 1] = calcCRC8(&exMessage[EXTEL_HEADER_TYPE_LEN], (exMessage[EXTEL_HEADER_TYPE_LEN]&0x3F));

    return lastItem;
}


void createExBusMessage(uint8_t *exBusMessage, uint8_t *exMessage, uint8_t exBusID)
{
    uint16_t crc16;

    exBusMessage[EXBUS_HEADER_PACKET_ID] = exBusID;
    exBusMessage[EXBUS_HEADER_SUBLEN] = (exMessage[EXTEL_HEADER_TYPE_LEN] & 0x3F) + 2;    // +2: startbyte & CRC8
    exBusMessage[EXBUS_HEADER_MSG_LEN] = 8 + exBusMessage[EXBUS_HEADER_SUBLEN];

    crc16 = calcCRC16(exBusMessage,exBusMessage[EXBUS_HEADER_MSG_LEN] - JETIEXBUS_CRC_LEN);
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 2] = crc16;
    exBusMessage[exBusMessage[EXBUS_HEADER_MSG_LEN] - 1] = crc16>>8;
}


void initJetiExBusTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    UNUSED(initialTelemetryConfig);

    // Init Ex Bus Frame header
    jetiExBusTelemetryFrame[EXBUS_HEADER_SYNC] = 0x3B;       // Startbytes
    jetiExBusTelemetryFrame[EXBUS_HEADER_REQ] = 0x01;
    jetiExBusTelemetryFrame[EXBUS_HEADER_DATA_ID] = 0x3A;    // Ex Telemetry


    // Init Ex Telemetry header
    jetiExTelemetry[EXTEL_HEADER_SYNC] = 0x9F;              // Startbyte
    jetiExTelemetry[EXTEL_HEADER_USN_LB] = 0x1E;            // Serial Number 4 Byte
    jetiExTelemetry[EXTEL_HEADER_USN_HB] = 0xA4;
    jetiExTelemetry[EXTEL_HEADER_LSN_LB] = 0x00;            // increment by telemetry count (%16) > only 15 values per device possible
    jetiExTelemetry[EXTEL_HEADER_LSN_HB] = 0x00;
    jetiExTelemetry[EXTEL_HEADER_RES] = 0x00;               // reserved, by default 0x00

    return;
}


void checkJetiExBusTelemetryState(void)
{
    return;
}


void handleJetiExBusTelemetry(void)
{
    if (jetiExBusRequestState == EXBUS_STATE_RECEIVED) {
        jetiExSensors[EX_VOLTAGE].value = vbat;
        jetiExSensors[EX_CURRENT].value = amperage;
        jetiExSensors[EX_ALTITUDE].value = BaroAlt;
        jetiExSensors[EX_CAPACITY].value = mAhDrawn;
        // switch to TX mode
        if (uartTotalRxBytesWaiting(jetiExBusPort) == 0) {
            serialSetMode(jetiExBusPort, MODE_TX);
            jetiExBusTransceiveState = EXBUS_TRANS_TX;
            sendJetiExBusTelemetry(jetiExBusSerialFrame[JETIEXBUS_FRAME_OFFSET + EXBUS_HEADER_PACKET_ID]);
            jetiExBusRequestState = EXBUS_STATE_PROCESSED;
        }
    }

    // check the state if transmit is ready
    if (jetiExBusTransceiveState == EXBUS_TRANS_TX_READY) {
        uartPort_t *s = (uartPort_t*)jetiExBusPort;
        if (uartTotalTxBytesFree(jetiExBusPort) == (s->port.txBufferSize - 1)) {     // workaround for 'isUartTransmitBufferEmpty()'
            serialSetMode(jetiExBusPort, MODE_RX);
            jetiExBusTransceiveState = EXBUS_TRANS_RX;
            jetiExBusRequestState = EXBUS_STATE_ZERO;
        }
    }
}


void sendJetiExBusTelemetry(uint8_t packetID)
{
    static uint8_t sensorDescriptionCounter = 0;
    static uint8_t sensorValueCounter = 1;
    static uint8_t requestLoop = 0;


    if (requestLoop == 100){              //every nth request send the name of a value
        if (sensorDescriptionCounter == JETI_EX_SENSOR_COUNT )
            sensorDescriptionCounter = 0;

        createExTelemetrieTextMessage(jetiExTelemetry, sensorDescriptionCounter, &jetiExSensors[sensorDescriptionCounter]);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetry, packetID);
        requestLoop = 0;
        sensorDescriptionCounter++;

    } else {
        if (sensorValueCounter >= JETI_EX_SENSOR_COUNT)
            sensorValueCounter = 1;

        sensorValueCounter = createExTelemetrieValueMessage(jetiExTelemetry, sensorValueCounter);
        createExBusMessage(jetiExBusTelemetryFrame, jetiExTelemetry, packetID);
    }

    for (uint8_t iCount = 0; iCount < jetiExBusTelemetryFrame[EXBUS_HEADER_MSG_LEN]; iCount++) {
        serialWrite(jetiExBusPort, jetiExBusTelemetryFrame[iCount]);
    }
    jetiExBusTransceiveState = EXBUS_TRANS_TX_READY;
    requestLoop++;
}

#endif //TELEMETRY
