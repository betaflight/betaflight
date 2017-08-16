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

#include <stdint.h>
#include <stdbool.h>

#include "config/parameter_group.h"
#include "drivers/serial.h"

typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0), // 1
    FUNCTION_GPS                 = (1 << 1), // 2
    FUNCTION_TELEMETRY_FRSKY     = (1 << 2), // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3), // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4), // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5), // 32
    FUNCTION_RX_SERIAL           = (1 << 6), // 64
    FUNCTION_BLACKBOX            = (1 << 7), // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 8), // 256
    FUNCTION_TELEMETRY_IBUS      = (1 << 9), // 512
    FUNCTION_RCSPLIT             = (1 << 10) // 1024
} serialPortFunction_e;

typedef enum {
    BAUD_AUTO = 0,
    BAUD_1200,
    BAUD_2400,
    BAUD_4800,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_460800,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000
} baudRate_e;

extern const uint32_t baudRates[];

// serial port identifiers are now fixed, these values are used by MSP commands.
typedef enum {
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_USART4,
    SERIAL_PORT_USART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_USB_VCP = 20,
    SERIAL_PORT_SOFTSERIAL1 = 30,
    SERIAL_PORT_SOFTSERIAL2,
    SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

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
    uint16_t functionMask;
    serialPortIdentifier_e identifier;
    uint8_t msp_baudrateIndex;
    uint8_t gps_baudrateIndex;
    uint8_t peripheral_baudrateIndex;
    uint8_t telemetry_baudrateIndex; // not used for all telemetry systems, e.g. HoTT only works at 19200.
} serialPortConfig_t;

typedef struct serialConfig_s {
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);

typedef void serialConsumer(uint8_t);

//
// configuration
//
void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable);
void serialRemovePort(serialPortIdentifier_e identifier);
uint8_t serialGetAvailablePortCount(void);
bool serialIsPortAvailable(serialPortIdentifier_e identifier);
bool isSerialConfigValid(const serialConfig_t *serialConfig);
serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier);
bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function);

portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function);
bool isSerialPortShared(const serialPortConfig_t *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction);

serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier);
int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier);
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
void serialEvaluateNonMspData(serialPort_t *serialPort, uint8_t receivedChar);
void serialPassthrough(serialPort_t *left, serialPort_t *right, serialConsumer *leftC, serialConsumer *rightC);
