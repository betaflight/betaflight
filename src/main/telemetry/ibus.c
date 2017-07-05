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

#include "platform.h"

#if defined(TELEMETRY) && defined(TELEMETRY_IBUS)

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"
#include "scheduler/scheduler.h"

#include "io/serial.h"

#include "sensors/barometer.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/gps.h"

#include "flight/imu.h"
#include "flight/failsafe.h"

#include "navigation/navigation.h"

#include "telemetry/ibus.h"
#include "telemetry/ibus_shared.h"
#include "telemetry/telemetry.h"
#include "fc/config.h"
#include "config/feature.h"

/*
 * iBus Telemetry is a half-duplex serial protocol. It shares 1 line for
 * both TX and RX. It runs at a fixed baud rate of 115200. Queries are sent
 * every 7ms by the iBus receiver. Multiple sensors can be daisy chained with
 * ibus but not with this implementation, only because i don't have one of the
 * sensors to test with!
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

static serialPort_t *ibusSerialPort = NULL;
static serialPortConfig_t *ibusSerialPortConfig;
static uint8_t outboundBytesToIgnoreOnRxCount = 0;
static bool ibusTelemetryEnabled = false;
static portSharing_e ibusPortSharing;

static uint8_t ibusReceiveBuffer[IBUS_RX_BUF_LEN] = { 0x0 };

static void pushOntoTail(uint8_t buffer[IBUS_MIN_LEN], size_t bufferLength, uint8_t value) {
    for (size_t i = 0; i < bufferLength - 1; i++) {
        buffer[i] = buffer[i + 1];
    }
    ibusReceiveBuffer[bufferLength - 1] = value;
}

void initIbusTelemetry(void) {
    ibusSerialPortConfig = findSerialPortConfig(FUNCTION_TELEMETRY_IBUS);
    uint8_t type = telemetryConfig()->ibusTelemetryType;
#if defined(GPS)
    if (type == 1 || type == 2) changeTypeIbusTelemetry(15, IBUS_MEAS_TYPE_SPE);
    if (type == 2) changeTypeIbusTelemetry(10, IBUS_MEAS_TYPE_ALT);
#endif
    ibusPortSharing = determinePortSharing(ibusSerialPortConfig, FUNCTION_TELEMETRY_IBUS);
    ibusTelemetryEnabled = false;
}

void handleIbusTelemetry(void) {
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
        if (ibusIsChecksumOkIa6b(ibusReceiveBuffer, IBUS_RX_BUF_LEN)) {
            outboundBytesToIgnoreOnRxCount += respondToIbusRequest(ibusReceiveBuffer);
        }
    }
}

bool checkIbusTelemetryState(void) {
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

void configureIbusTelemetryPort(void) {
    if (!ibusSerialPortConfig) {
        return;
    }
    if (isSerialPortShared(ibusSerialPortConfig, FUNCTION_RX_SERIAL, FUNCTION_TELEMETRY_IBUS)) {
        // serialRx will open port and handle telemetry
        return;
    }
    ibusSerialPort = openSerialPort(ibusSerialPortConfig->identifier, FUNCTION_TELEMETRY_IBUS, NULL, IBUS_BAUDRATE, MODE_RXTX, SERIAL_BIDIR | SERIAL_NOT_INVERTED);
    if (!ibusSerialPort) {
        return;
    }
    initSharedIbusTelemetry(ibusSerialPort);
    ibusTelemetryEnabled = true;
    outboundBytesToIgnoreOnRxCount = 0;
}

void freeIbusTelemetryPort(void) {
    closeSerialPort(ibusSerialPort);
    ibusSerialPort = NULL;

    ibusTelemetryEnabled = false;
}

#endif
