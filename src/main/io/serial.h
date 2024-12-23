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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "pg/pg.h"
#include "drivers/serial.h"

typedef enum {
    PORTSHARING_UNUSED = 0,
    PORTSHARING_NOT_SHARED,
    PORTSHARING_SHARED
} portSharing_e;

typedef enum {
    FUNCTION_NONE                = 0,
    FUNCTION_MSP                 = (1 << 0),  // 1
    FUNCTION_GPS                 = (1 << 1),  // 2
    FUNCTION_TELEMETRY_FRSKY_HUB = (1 << 2),  // 4
    FUNCTION_TELEMETRY_HOTT      = (1 << 3),  // 8
    FUNCTION_TELEMETRY_LTM       = (1 << 4),  // 16
    FUNCTION_TELEMETRY_SMARTPORT = (1 << 5),  // 32
    FUNCTION_RX_SERIAL           = (1 << 6),  // 64
    FUNCTION_BLACKBOX            = (1 << 7),  // 128
    FUNCTION_TELEMETRY_MAVLINK   = (1 << 9),  // 512
    FUNCTION_ESC_SENSOR          = (1 << 10), // 1024
    FUNCTION_VTX_SMARTAUDIO      = (1 << 11), // 2048
    FUNCTION_TELEMETRY_IBUS      = (1 << 12), // 4096
    FUNCTION_VTX_TRAMP           = (1 << 13), // 8192
    FUNCTION_RCDEVICE            = (1 << 14), // 16384
    FUNCTION_LIDAR_TF            = (1 << 15), // 32768
    FUNCTION_FRSKY_OSD           = (1 << 16), // 65536
    FUNCTION_VTX_MSP             = (1 << 17), // 131072
    FUNCTION_GIMBAL              = (1 << 18), // 262144
} serialPortFunction_e;

#define TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK (FUNCTION_TELEMETRY_FRSKY_HUB | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_MAVLINK)
#define TELEMETRY_PORT_FUNCTIONS_MASK (TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_SMARTPORT)

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000,
    BAUD_COUNT
} baudRate_e;

extern const uint32_t baudRates[BAUD_COUNT];

// serial port identifiers are now fixed, these values are used by MSP commands.
typedef enum {
    SERIAL_PORT_ALL = -2,
    SERIAL_PORT_NONE = -1,
    SERIAL_PORT_START_INDEX = 0,
#if SERIAL_UART_FIRST_INDEX == 0
    SERIAL_PORT_UART_FIRST = 50,
    SERIAL_PORT_UART0 = SERIAL_PORT_UART_FIRST,
    SERIAL_PORT_USART1,
#else
    SERIAL_PORT_UART_FIRST = SERIAL_PORT_START_INDEX,
    SERIAL_PORT_USART1 = SERIAL_PORT_UART_FIRST,
#endif
    SERIAL_PORT_UART1 = SERIAL_PORT_USART1,
    SERIAL_PORT_USART2,
    SERIAL_PORT_UART2 = SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_UART3 = SERIAL_PORT_USART3,
    SERIAL_PORT_UART4,
    SERIAL_PORT_UART5,
    SERIAL_PORT_USART6,
    SERIAL_PORT_UART6 = SERIAL_PORT_USART6,
    SERIAL_PORT_USART7,
    SERIAL_PORT_UART7 = SERIAL_PORT_USART7,
    SERIAL_PORT_USART8,
    SERIAL_PORT_UART8 = SERIAL_PORT_USART8,
    SERIAL_PORT_UART9,
    SERIAL_PORT_USART10,
    SERIAL_PORT_UART10 = SERIAL_PORT_USART10,

    SERIAL_PORT_USB_VCP = 20,

    SERIAL_PORT_SOFTSERIAL_FIRST = 30,
    SERIAL_PORT_SOFTSERIAL1 = SERIAL_PORT_SOFTSERIAL_FIRST,
    SERIAL_PORT_SOFTSERIAL2,

    SERIAL_PORT_LPUART_FIRST = 40,
    SERIAL_PORT_LPUART1 = SERIAL_PORT_LPUART_FIRST,
} serialPortIdentifier_e;

// use value from target serial port normalization
#define SERIAL_PORT_COUNT (SERIAL_UART_COUNT + SERIAL_LPUART_COUNT + SERIAL_SOFTSERIAL_COUNT + SERIAL_VCP_COUNT)

extern const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT];

typedef enum {
    SERIALTYPE_INVALID = -1,
    SERIALTYPE_USB_VCP = 0,
    SERIALTYPE_UART,
    SERIALTYPE_LPUART,
    SERIALTYPE_SOFTSERIAL,
    SERIALTYPE_COUNT
} serialType_e;

//
// runtime
//
typedef struct serialPortUsage_s {
    serialPort_t *serialPort;
    serialPortFunction_e function;
    serialPortIdentifier_e identifier;
} serialPortUsage_t;

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction);

//
// configuration
//
typedef struct serialPortConfig_s {
    uint32_t functionMask;
    int8_t identifier;
    uint8_t msp_baudrateIndex;
    uint8_t gps_baudrateIndex;
    uint8_t blackbox_baudrateIndex;
    uint8_t telemetry_baudrateIndex; // not used for all telemetry systems, e.g. HoTT only works at 19200.
} serialPortConfig_t;

typedef struct serialConfig_s {
    serialPortConfig_t portConfigs[SERIAL_PORT_COUNT];
    uint16_t serial_update_rate_hz;
    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

PG_DECLARE(serialConfig_t, serialConfig);

typedef void serialConsumer(uint8_t);

//
// configuration
//
void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable);
void serialRemovePort(serialPortIdentifier_e identifier);
bool serialIsPortAvailable(serialPortIdentifier_e identifier);
bool isSerialConfigValid(serialConfig_t *serialConfig);
const serialPortConfig_t *serialFindPortConfiguration(serialPortIdentifier_e identifier);
serialPortConfig_t *serialFindPortConfigurationMutable(serialPortIdentifier_e identifier);
bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function);
const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function);

portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function);
bool isSerialPortShared(const serialPortConfig_t *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction);

void pgResetFn_serialConfig(serialConfig_t *serialConfig); //!!TODO remove need for this
serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier);
int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier);
serialPortIdentifier_e findSerialPortByName(const char* portName, int (*cmp)(const char *portName, const char *candidate));
const char* serialName(serialPortIdentifier_e identifier, const char* notFound);

//
// runtime
//
serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr rxCallback,
    void *rxCallbackData,
    uint32_t baudrate,
    portMode_e mode,
    portOptions_e options
);
void closeSerialPort(serialPort_t *serialPort);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

baudRate_e lookupBaudRateIndex(uint32_t baudRate);

serialType_e serialType(serialPortIdentifier_e identifier);

// resource index of given identifier, or -1 if not available
int serialResourceIndex(serialPortIdentifier_e identifier);

//
// msp/cli/bootloader
//
void serialPassthrough(serialPort_t *left, serialPort_t *right, serialConsumer *leftC, serialConsumer *rightC);
