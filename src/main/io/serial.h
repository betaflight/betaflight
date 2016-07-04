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

#pragma once

typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP_SERVER          = (1 << 0), // 1
    FUNCTION_GPS                 = (1 << 1), // 2
    FUNCTION_TELEMETRY_FRSKY     = (1 << 2), // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3), // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4), // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5), // 32
    FUNCTION_RX_SERIAL           = (1 << 6), // 64
    FUNCTION_BLACKBOX            = (1 << 7), // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 8), // 256
    FUNCTION_MSP_CLIENT          = (1 << 9)  // 512
} serialPortFunction_e;

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
} baudRate_e;

extern const uint32_t baudRates[];

// serial port identifiers are now fixed, these values are used by MSP commands.
typedef enum {
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_UART1 = 0,
    SERIAL_PORT_UART2,
    SERIAL_PORT_UART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

void serialInit(bool softserialEnabled);

//
// runtime
//

typedef struct serialPortUsage_s {
    serialPortIdentifier_e identifier;
    serialPort_t *serialPort;
    serialPortFunction_e function;
} serialPortUsage_t;

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);
serialPort_t *findNextSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);

//
// configuration
//

#define FUNCTION_BAUD_RATE_COUNT 4 // FIXME OSD only needs 2, FC needs 4.

typedef struct serialPortConfig_s {
    serialPortIdentifier_e identifier;
    uint16_t functionMask;
    uint8_t baudRates[FUNCTION_BAUD_RATE_COUNT];
} serialPortConfig_t;

typedef struct serialConfig_s {
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);

//
// configuration
//
void serialRemovePort(serialPortIdentifier_e identifier);
uint8_t serialGetAvailablePortCount(void);
bool serialIsPortAvailable(serialPortIdentifier_e identifier);
bool isSerialConfigValid(serialConfig_t *serialConfig);
serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier);
bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
serialPortConfig_t *findSerialPortConfig(uint16_t mask);
serialPortConfig_t *findNextSerialPortConfig(uint16_t mask);

portSharing_e determinePortSharing(serialPortConfig_t *portConfig, serialPortFunction_e function);
bool isSerialPortShared(serialPortConfig_t *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction);
bool isSerialPortOpen(serialPortConfig_t *portConfig);


//
// runtime
//
serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    uint32_t baudrate,
    portMode_t mode,
    portOptions_t options
);
void closeSerialPort(serialPort_t *serialPort);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

baudRate_e lookupBaudRateIndex(uint32_t baudRate);


//
// msp/cli/bootloader
//
void evaluateOtherData(serialPort_t *serialPort, uint8_t receivedChar);
void handleSerial(void);
