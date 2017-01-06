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
 * FlySky iBus telemetry implementation by CraigJPerry.
 * Unit tests and some additions by Unitware
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "common/utils.h"

#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)

#include "config/config_master.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/serial.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"

#include "io/serial.h"
#include "fc/rc_controls.h"
#include "scheduler/scheduler.h"

#include "telemetry/telemetry.h"    
#include "telemetry/ibus.h"

/*
 * iBus Telemetry is a half-duplex serial protocol. It shares 1 line for
 * both TX and RX. It runs at a fixed baud rate of 115200. Queries are sent
 * every 7ms by the iBus receiver. Multiple sensors can be daisy chained with
 * ibus but this is implemented but not tested because i don't have one of the
 * sensors to test with
 *
 *     _______
 *    /       \                                             /---------\
 *    | STM32 |--UART TX-->[Bi-directional @ 115200 baud]<--| IBUS RX |
 *    |  uC   |--UART RX--x[not connected]                  \---------/
 *    \_______/
 *
 *
 * The protocol is driven by the iBus receiver, currently either an IA6B or
 * IA10. All iBus traffic is little endian. It begins with the iBus rx
 * querying for a sensor on the iBus:
 *
 *
 *  /---------\
 *  | IBUS RX | > Hello sensor at address 1, are you there?
 *  \---------/     [ 0x04, 0x81, 0x7A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0x81       - bits 7-4 Command (1000 = discover sensor)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x7A, 0xFF - Checksum, 0xFFFF - (0x04 + 0x81)
 *
 *
 * Due to the daisy-chaining, this hello also serves to inform the sensor
 * of its address (position in the chain). There are 16 possible addresses
 * in iBus, however address 0 is reserved for the rx's internally measured
 * voltage leaving 0x1 to 0xF remaining.
 *
 * Having learned it's address, the sensor simply echos the message back:
 *
 *
 *                                                      /--------\
 *                              Yes, i'm here, hello! < | Sensor |
 *                       [ 0x04, 0x81, 0x7A, 0xFF ]     \--------/
 *
 *     0x04, 0x81, 0x7A, 0xFF - Echo back received packet
 *
 *
 * On receipt of a response, the iBus rx next requests the sensor's type:
 *
 *
 *  /---------\
 *  | IBUS RX | > Sensor at address 1, what type are you?
 *  \---------/     [ 0x04, 0x91, 0x6A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0x91       - bits 7-4 Command (1001 = request sensor type)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x6A, 0xFF - Checksum, 0xFFFF - (0x04 + 0x91)
 *
 *
 * To which the sensor responds with its details:
 *
 *
 *                                                      /--------\
 *                              Yes, i'm here, hello! < | Sensor |
 *                [ 0x06 0x91 0x00 0x02 0x66 0xFF ]     \--------/
 *
 *     0x06       - Packet Length
 *     0x91       - bits 7-4 Command (1001 = request sensor type)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x00       - Measurement type (0 = internal voltage)
 *     0x02       - Unknown, always 0x02
 *     0x66, 0xFF - Checksum, 0xFFFF - (0x06 + 0x91 + 0x00 + 0x02)
 *
 *
 * The iBus rx continues the discovery process by querying the next
 * address. Discovery stops at the first address which does not respond.
 *
 * The iBus rx then begins a continual loop, requesting measurements from
 * each sensor discovered:
 *
 *
 *  /---------\
 *  | IBUS RX | > Sensor at address 1, please send your measurement
 *  \---------/     [ 0x04, 0xA1, 0x5A, 0xFF ]
 *
 *     0x04       - Packet Length
 *     0xA1       - bits 7-4 Command (1010 = request measurement)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x5A, 0xFF - Checksum, 0xFFFF - (0x04 + 0xA1)
 *
 *
 *                                                      /--------\
 *                                I'm reading 0 volts < | Sensor |
 *                [ 0x06 0xA1 0x00 0x00 0x5E 0xFF ]     \--------/
 *
 *     0x06       - Packet Length
 *     0xA1       - bits 7-4 Command (1010 = request measurement)
 *                  bits 3-0 Address (0001 = address 1)
 *     0x00, 0x00 - The measurement
 *     0x58, 0xFF - Checksum, 0xFFFF - (0x06 + 0xA1 + 0x00 + 0x00)
 *
 *
 * Due to the limited telemetry data types possible with ibus, we
 * simply send everything which can be represented. Currently this
 * is voltage and temperature.
 *
 */

/*
PG_REGISTER_WITH_RESET_TEMPLATE(ibusTelemetryConfig_t, ibusTelemetryConfig, PG_IBUS_TELEMETRY_CONFIG, 0);

PG_RESET_TEMPLATE(ibusTelemetryConfig_t, ibusTelemetryConfig,
                  .report_cell_voltage = false,
                 );
*/

#define IBUS_TASK_PERIOD_US (500)

#define IBUS_UART_MODE     (MODE_RXTX)
#define IBUS_BAUDRATE      (115200)
#define IBUS_CYCLE_TIME_MS (8)

#define IBUS_CHECKSUM_SIZE (2)

#define IBUS_MIN_LEN       (2 + IBUS_CHECKSUM_SIZE)
#define IBUS_MAX_TX_LEN    (6)
#define IBUS_MAX_RX_LEN    (4)
#define IBUS_RX_BUF_LEN    (IBUS_MAX_RX_LEN)

#define IBUS_TEMPERATURE_OFFSET  (400)


typedef uint8_t ibusAddress_t;

typedef enum {
    IBUS_COMMAND_DISCOVER_SENSOR      = 0x80,
    IBUS_COMMAND_SENSOR_TYPE          = 0x90,
    IBUS_COMMAND_MEASUREMENT          = 0xA0
} ibusCommand_e;

typedef enum {
    IBUS_SENSOR_TYPE_TEMPERATURE      = 0x01,
    IBUS_SENSOR_TYPE_RPM              = 0x02,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE = 0x03
} ibusSensorType_e;

/* Address lookup relative to the sensor base address which is the lowest address seen by the FC 
   The actual lowest value is likely to change when sensors are daisy chained */
static const uint8_t sensorAddressTypeLookup[] = {
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE,
    IBUS_SENSOR_TYPE_TEMPERATURE,
    IBUS_SENSOR_TYPE_RPM
};

static serialPort_t *ibusSerialPort = NULL;
static serialPortConfig_t *ibusSerialPortConfig;

/* The sent bytes will be echoed back since Tx and Rx are wired together, this counter
 * will keep track of how many rx chars that shall be discarded */
static uint8_t outboundBytesToIgnoreOnRxCount = 0;

static bool ibusTelemetryEnabled = false;
static portSharing_e ibusPortSharing;

#define INVALID_IBUS_ADDRESS 0
static ibusAddress_t ibusBaseAddress = INVALID_IBUS_ADDRESS;

static uint8_t ibusReceiveBuffer[IBUS_RX_BUF_LEN] = { 0x0 };

static uint16_t calculateChecksum(const uint8_t *ibusPacket, size_t packetLength)
{
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < packetLength - IBUS_CHECKSUM_SIZE; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

static bool isChecksumOk(const uint8_t *ibusPacket)
{
    uint16_t calculatedChecksum = calculateChecksum(ibusReceiveBuffer, IBUS_RX_BUF_LEN);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[IBUS_RX_BUF_LEN - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[IBUS_RX_BUF_LEN - 2];
}

static void transmitIbusPacket(uint8_t *ibusPacket, size_t payloadLength)
{
    uint16_t checksum = calculateChecksum(ibusPacket, payloadLength + IBUS_CHECKSUM_SIZE);
    for (size_t i = 0; i < payloadLength; i++) {
        serialWrite(ibusSerialPort, ibusPacket[i]);
    }
    serialWrite(ibusSerialPort, checksum & 0xFF);
    serialWrite(ibusSerialPort, checksum >> 8);
    outboundBytesToIgnoreOnRxCount += payloadLength + IBUS_CHECKSUM_SIZE;
}

static void sendIbusDiscoverSensorReply(ibusAddress_t address)
{
    uint8_t sendBuffer[] = { 0x04, IBUS_COMMAND_DISCOVER_SENSOR | address};
    transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static void sendIbusSensorType(ibusAddress_t address)
{
    uint8_t sendBuffer[] = {0x06,
                            IBUS_COMMAND_SENSOR_TYPE | address,
                            sensorAddressTypeLookup[address - ibusBaseAddress],
                            0x02
                           };
    transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static void sendIbusMeasurement(ibusAddress_t address, uint16_t measurement)
{
    uint8_t sendBuffer[] = { 0x06, IBUS_COMMAND_MEASUREMENT | address, measurement & 0xFF, measurement >> 8};
    transmitIbusPacket(sendBuffer, sizeof(sendBuffer));
}

static bool isCommand(ibusCommand_e expected, const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0xF0) == expected;
}

static ibusAddress_t getAddress(const uint8_t *ibusPacket)
{
    return (ibusPacket[1] & 0x0F);
}

static void dispatchMeasurementReply(ibusAddress_t address)
{
    int value;
    
    switch (sensorAddressTypeLookup[address - ibusBaseAddress]) {
    case IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE:
        value = getVbat() * 10;
        if (ibusTelemetryConfig()->report_cell_voltage) {
            value /= batteryCellCount;
        }
        sendIbusMeasurement(address, value);
        break;

    case IBUS_SENSOR_TYPE_TEMPERATURE:
        #ifdef BARO
            value = (baro.baroTemperature + 5) / 10; // +5 to make integer division rounding correct
        #else
            value = telemTemperature1 * 10;
        #endif
        sendIbusMeasurement(address, value + IBUS_TEMPERATURE_OFFSET);
        break;

    case IBUS_SENSOR_TYPE_RPM:
        sendIbusMeasurement(address, (uint16_t) rcCommand[THROTTLE]);
        break;
    }
}

static void autodetectFirstReceivedAddressAsBaseAddress(ibusAddress_t returnAddress)
{
    if ((INVALID_IBUS_ADDRESS == ibusBaseAddress) &&
        (INVALID_IBUS_ADDRESS != returnAddress)) {
        ibusBaseAddress = returnAddress;
    }
}

static bool theAddressIsWithinOurRange(ibusAddress_t returnAddress)
{
    return (returnAddress >= ibusBaseAddress) &&
           (ibusAddress_t)(returnAddress - ibusBaseAddress) < ARRAYLEN(sensorAddressTypeLookup);
}

static void respondToIbusRequest(uint8_t ibusPacket[static IBUS_RX_BUF_LEN])
{
    ibusAddress_t returnAddress = getAddress(ibusPacket);

    autodetectFirstReceivedAddressAsBaseAddress(returnAddress);

    if (theAddressIsWithinOurRange(returnAddress)) {
        if (isCommand(IBUS_COMMAND_DISCOVER_SENSOR, ibusPacket)) {
            sendIbusDiscoverSensorReply(returnAddress);
        } else if (isCommand(IBUS_COMMAND_SENSOR_TYPE, ibusPacket)) {
            sendIbusSensorType(returnAddress);
        } else if (isCommand(IBUS_COMMAND_MEASUREMENT, ibusPacket)) {
            dispatchMeasurementReply(returnAddress);
        }
    }
}

static void pushOntoTail(uint8_t buffer[IBUS_MIN_LEN], size_t bufferLength, uint8_t value)
{
    memmove(buffer, buffer + 1, bufferLength - 1);
    ibusReceiveBuffer[bufferLength - 1] = value;
}

void initIbusTelemetry(void)
{
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibusPortSharing = determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibusBaseAddress = INVALID_IBUS_ADDRESS;
}

void handleIbusTelemetry(void)
{
    if (!ibusTelemetryEnabled) {
        return;
    }

    while (serialRxBytesWaiting(ibusSerialPort) > 0) {
        uint8_t c = serialRead(ibusSerialPort);

        if (outboundBytesToIgnoreOnRxCount) {
            outboundBytesToIgnoreOnRxCount--;
            continue;
        }

        pushOntoTail(ibusReceiveBuffer, IBUS_RX_BUF_LEN, c);

        if (isChecksumOk(ibusReceiveBuffer)) {
            respondToIbusRequest(ibusReceiveBuffer);
        }
    }
}

bool checkIbusTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ibusPortSharing);

    if (newTelemetryEnabledValue == ibusTelemetryEnabled) {
        return false;
    }

    if (newTelemetryEnabledValue) {
        rescheduleTask(TASK_TELEMETRY, IBUS_TASK_PERIOD_US);
        configureIbusTelemetryPort();
    } else {
        freeIbusTelemetryPort();
    }

    return true;
}

void configureIbusTelemetryPort(void)
{
    portOptions_t portOptions;

    if (!ibusSerialPortConfig) {
        return;
    }

    portOptions = SERIAL_BIDIR;

    ibusSerialPort = openSerialPort(ibusSerialPortConfig->identifier, FUNCTION_TELEMETRY_IBUS, NULL, IBUS_BAUDRATE,
                                    IBUS_UART_MODE, portOptions);

    if (!ibusSerialPort) {
        return;
    }

    ibusTelemetryEnabled = true;
    outboundBytesToIgnoreOnRxCount = 0;
}

void freeIbusTelemetryPort(void)
{
    closeSerialPort(ibusSerialPort);
    ibusSerialPort = NULL;

    ibusTelemetryEnabled = false;
}

#endif