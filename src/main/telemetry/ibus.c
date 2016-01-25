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
 *
 * Many thanks to Dave Borthwick's iBus telemetry dongle converter for
 * PIC 12F1572 (also distributed under GPLv3) which was referenced to
 * clarify the protocol.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef TELEMETRY

#include "drivers/serial.h"

#include "io/serial.h"

#include "sensors/battery.h"

#include "telemetry/telemetry.h"
#include "telemetry/ibus.h"

/*
 * iBus Telemetry is a half-duplex serial protocol. It shares 1 line for
 * both TX and RX. It runs at a fixed baud rate of 115200. Queries are sent
 * every 7ms by the iBus receiver. Multiple sensors can be daisy chained.
 * Cleanflight must be the last sensor in the chain.
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
 *     0x5E, 0xFF - Checksum, 0xFF - (0x06 + 0xA1 + 0x00 + 0x00)
 *
 *
 * Due to the limited telemetry data types possible with ibus, we
 * simply send everything which can be represented. Currently this
 * is voltage and temperature.
 *
 */

#define IBUS_UART_MODE (MODE_RXTX)
#define IBUS_BAUDRATE (115200)
#define IBUS_CYCLE_TIME_MS (7)

typedef enum
{
    VOLTAGE          = 0x1,
    GYRO_TEMP        = 0x2
} ibusAddress_e;

typedef enum
{
    DISCOVER_SENSOR  = 0x80,
    SENSOR_TYPE      = 0x90,
    MEASUREMENT      = 0xA0
} ibusQuery_e;

typedef enum
{
    INTERNAL_VOLTAGE = 0x00,
    TEMPERATURE      = 0x01,
    RPM              = 0x02,
    EXTERNAL_VOLTAGE = 0x03
} ibusMeasurementType_e;

static serialPort_t *ibusSerialPort = NULL;
static serialPortConfig_t *ibusSerialPortConfig;

static telemetryConfig_t *telemetryConfig;
static bool ibusTelemetryEnabled =  false;
static portSharing_e ibusPortSharing;

extern int16_t telemTemperature1;

#define IBUS_MAX_REQUEST_LEN (4)
static uint8_t ibusReceiveBuffer[IBUS_MAX_REQUEST_LEN] = {0};

#define IBUS_CHECKSUM_SIZE (2)
static uint8_t *checksum(uint8_t iBusPacket[IBUS_MAX_REQUEST_LEN])
{
    uint16_t checksum = 0xFFFF;
    const uint8_t packetLength = iBusPacket[0];

    for(uint8_t i = 0; i < packetLength - IBUS_CHECKSUM_SIZE; i++) {
        checksum -= iBusPacket[i];
    }

    uint8_t *result = calloc(2, sizeof(uint8_t));
    result[0] = checksum & 0xFF;
    result[1] = checksum >> 8;
    return result;
}

static bool checksumValid(uint8_t iBusPacket[IBUS_MAX_REQUEST_LEN])
{
    uint8_t *result = checksum(iBusPacket);
    return memcmp(result, iBusPacket + IBUS_MAX_REQUEST_LEN - 2, 2) == 0;
}

static void sendIbusPacket(ibusMeasurementType_e measurementType, uint8_t address, uint16_t value)
{
    uint8_t sendBuffer[] = {0x06, measurementType & address, value & 0xF, value >> 8, 0x0, 0x0};
    uint8_t *csum = checksum(sendBuffer);
    memcpy(csum, sendBuffer + 4, 2);
    for(size_t i = 0; i < sizeof sendBuffer; i++) {
        serialWrite(ibusSerialPort, sendBuffer[i]);
    }
}

static void replyWithTemperature(int16_t temperature)
{
    sendIbusPacket(TEMPERATURE, ibusReceiveBuffer[1] & 0xF, (uint16_t) temperature);
}

static void replyWithVoltage(uint16_t voltage)
{
    sendIbusPacket(INTERNAL_VOLTAGE, ibusReceiveBuffer[1] & 0xF, voltage);
}

static bool addressIs(ibusAddress_e address)
{
    return (ibusReceiveBuffer[1] & 0xF) == address;
}

static void respond()
{
    if (addressIs(VOLTAGE)) {
        replyWithVoltage(vbat);
    } else if (addressIs(GYRO_TEMP)) {
        replyWithTemperature(telemTemperature1);
    }
}

static bool validRequestInBuffer()
{
    if(ibusReceiveBuffer[0] == 0x04 && checksumValid(ibusReceiveBuffer)) {
        return true;
    }
    return false;
}

static void pushOntoIbusBufferTail(uint16_t c)
{
    for(size_t i = 0; i < IBUS_MAX_REQUEST_LEN - 1; i++) {
        ibusReceiveBuffer[i] = ibusReceiveBuffer[i+1];
    }
    ibusReceiveBuffer[IBUS_MAX_REQUEST_LEN - 1] = c;
}

void initIbusTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    ibusPortSharing = determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
}

void handleIbusTelemetry(void)
{
    while (serialRxBytesWaiting(ibusSerialPort) > 0) {
        uint8_t c = serialRead(ibusSerialPort);
        pushOntoIbusBufferTail(c);
        if (validRequestInBuffer()) {
            respond();
        }
    }
}

void checkIbusTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(ibusPortSharing);

    if (newTelemetryEnabledValue == ibusTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue) {
            configureIbusTelemetryPort();
    } else {
            freeIbusTelemetryPort();
    }
}

void configureIbusTelemetryPort(void)
{
    portOptions_t portOptions;

    if (!ibusSerialPortConfig) {
        return;
    }

    portOptions = SERIAL_BIDIR;

    ibusSerialPort = openSerialPort(ibusSerialPortConfig->identifier, FUNCTION_TELEMETRY_IBUS, NULL, IBUS_BAUDRATE, IBUS_UART_MODE, portOptions);

    if (!ibusSerialPort) {
        return;
    }

    ibusTelemetryEnabled = true;
}

void freeIbusTelemetryPort(void)
{
    closeSerialPort(ibusSerialPort);
    ibusSerialPort = NULL;

    ibusTelemetryEnabled = false;
}

#endif
