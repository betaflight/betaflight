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
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0),
    FUNCTION_CLI                 = (1 << 1),
    FUNCTION_FRSKY_TELEMETRY     = (1 << 2),
    FUNCTION_HOTT_TELEMETRY      = (1 << 3),
    FUNCTION_MSP_TELEMETRY       = (1 << 4),
    FUNCTION_SMARTPORT_TELEMETRY = (1 << 5),
    FUNCTION_SERIAL_RX           = (1 << 6),
    FUNCTION_GPS                 = (1 << 7),
    FUNCTION_BLACKBOX            = (1 << 8)
} serialPortFunction_e;

// serial port identifiers are now fixed, these values are used by MSP commands.
typedef enum {
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_USART4,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

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

typedef struct serialPortConfig_s {
    serialPortIdentifier_e identifier;
    serialPortFunction_e functionMask;
    uint32_t baudrate; // not used for all functions, e.g. HoTT only works at 19200.
} serialPortConfig_t;

typedef struct serialConfig_s {
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
} serialConfig_t;


//
// configuration
//
bool isSerialConfigValid(serialConfig_t *serialConfig);
bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function);


//
// runtime
//
serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    uint32_t baudrate,
    portMode_t mode,
    serialInversion_e inversion
);
void closeSerialPort(serialPort_t *serialPort);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

//
// msp/cli/bootloader
//
void evaluateOtherData(serialPort_t *serialPort, uint8_t receivedChar);
void handleSerial(void);
