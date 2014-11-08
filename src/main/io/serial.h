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
    FUNCTION_NONE               = 0,
    FUNCTION_MSP                = (1 << 0),
    FUNCTION_CLI                = (1 << 1),
    FUNCTION_TELEMETRY          = (1 << 2),
    FUNCTION_SERIAL_RX          = (1 << 3),
    FUNCTION_GPS                = (1 << 4),
    FUNCTION_GPS_PASSTHROUGH    = (1 << 5)
} serialPortFunction_e;

typedef enum {
    NO_AUTOBAUD = 0,
    AUTOBAUD
} autoBaud_e;

typedef struct functionConstraint_s {
    serialPortFunction_e function;
    uint32_t minBaudRate;
    uint32_t maxBaudRate;
    autoBaud_e autoBaud;
    uint8_t requiredSerialPortFeatures;
} functionConstraint_t;

typedef enum {
    SCENARIO_UNUSED                             = FUNCTION_NONE,

    SCENARIO_CLI_ONLY                           = FUNCTION_CLI,
    SCENARIO_GPS_ONLY                           = FUNCTION_GPS,
    SCENARIO_GPS_PASSTHROUGH_ONLY               = FUNCTION_GPS_PASSTHROUGH,
    SCENARIO_MSP_ONLY                           = FUNCTION_MSP,
    SCENARIO_MSP_CLI_GPS_PASTHROUGH             = FUNCTION_CLI | FUNCTION_MSP | FUNCTION_GPS_PASSTHROUGH,
    SCENARIO_MSP_CLI_TELEMETRY_GPS_PASTHROUGH   = FUNCTION_MSP | FUNCTION_CLI | FUNCTION_TELEMETRY | FUNCTION_GPS_PASSTHROUGH,
    SCENARIO_SERIAL_RX_ONLY                     = FUNCTION_SERIAL_RX,
    SCENARIO_TELEMETRY_ONLY                     = FUNCTION_TELEMETRY,
} serialPortFunctionScenario_e;

#define SERIAL_PORT_SCENARIO_COUNT 9
#define SERIAL_PORT_SCENARIO_MAX (SERIAL_PORT_SCENARIO_COUNT - 1)
extern const serialPortFunctionScenario_e serialPortScenarios[SERIAL_PORT_SCENARIO_COUNT];

typedef enum {
    SERIAL_PORT_1 = 0,
    SERIAL_PORT_2,
#if (SERIAL_PORT_COUNT > 2)
    SERIAL_PORT_3,
#if (SERIAL_PORT_COUNT > 3)
    SERIAL_PORT_4,
#if (SERIAL_PORT_COUNT > 4)
    SERIAL_PORT_5
#endif
#endif
#endif
} serialPortIndex_e;


#ifdef STM32F303xC

typedef enum {
    SERIAL_PORT_USB_VCP = 0,
    SERIAL_PORT_USART1,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_USART4
} serialPortIdentifier_e;

#define SERIAL_PORT_IDENTIFIER_COUNT 5
#else

#ifdef CCD3

typedef enum {
    SERIAL_PORT_USART1,
    SERIAL_PORT_USART3,
    SERIAL_PORT_SOFTSERIAL1,
    SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

#define SERIAL_PORT_IDENTIFIER_COUNT 4
#else

typedef enum {
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_USART3,
    SERIAL_PORT_SOFTSERIAL1,
    SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

#define SERIAL_PORT_IDENTIFIER_COUNT 5
#endif
#endif

// bitmask
typedef enum {
    SPF_NONE                   = 0,
    SPF_SUPPORTS_CALLBACK      = (1 << 0),
    SPF_SUPPORTS_SBUS_MODE     = (1 << 1),
    SPF_IS_SOFTWARE_INVERTABLE = (1 << 2)
} serialPortFeature_t;

typedef struct serialPortConstraint_s {
    const serialPortIdentifier_e identifier;
    uint32_t minBaudRate;
    uint32_t maxBaudRate;
    serialPortFeature_t feature;
} serialPortConstraint_t;

typedef struct serialPortFunction_s {
    serialPortIdentifier_e identifier;
    serialPort_t *port; // a NULL values indicates the port has not been opened yet.
    serialPortFunctionScenario_e scenario;
    serialPortFunction_e currentFunction;
} serialPortFunction_t;

typedef struct serialPortFunctionList_s {
    uint8_t serialPortCount;
    serialPortFunction_t *functions;
} serialPortFunctionList_t;

typedef struct serialConfig_s {
    uint8_t serial_port_scenario[SERIAL_PORT_COUNT];
    uint32_t msp_baudrate;
    uint32_t cli_baudrate;
    uint32_t gps_baudrate;
    uint32_t gps_passthrough_baudrate;

    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

uint8_t lookupScenarioIndex(serialPortFunctionScenario_e scenario);

serialPort_t *findOpenSerialPort(uint16_t functionMask);
serialPort_t *openSerialPort(serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, serialInversion_e inversion);

bool canOpenSerialPort(serialPortFunction_e function);
void beginSerialPortFunction(serialPort_t *port, serialPortFunction_e function);
void endSerialPortFunction(serialPort_t *port, serialPortFunction_e function);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

void applySerialConfigToPortFunctions(serialConfig_t *serialConfig);
bool isSerialConfigValid(serialConfig_t *serialConfig);
bool doesConfigurationUsePort(serialPortIdentifier_e portIdentifier);
bool isSerialPortFunctionShared(serialPortFunction_e functionToUse, uint16_t functionMask);
serialPort_t *findSharedSerialPort(serialPortFunction_e functionToUse, uint16_t functionMask);

const serialPortFunctionList_t *getSerialPortFunctionList(void);

void evaluateOtherData(uint8_t sr);
void handleSerial(void);
