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

#define SERIAL_PORT_COUNT 4

typedef enum {
    SERIAL_PORT_1 = 0,
    SERIAL_PORT_2,
    SERIAL_PORT_3,
    SERIAL_PORT_4
} serialPortIndex_e;

typedef enum {
    SERIAL_PORT_USART1 = 0,
    SERIAL_PORT_USART2,
    SERIAL_PORT_SOFTSERIAL1,
    SERIAL_PORT_SOFTSERIAL2
} serialPortIdentifier_e;

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

typedef struct serialConfig_s {
    uint8_t serial_port_1_scenario;
    uint8_t serial_port_2_scenario;
    uint8_t serial_port_3_scenario;
    uint8_t serial_port_4_scenario;

    uint32_t msp_baudrate;
    uint32_t cli_baudrate;
    uint32_t gps_passthrough_baudrate;

    uint32_t hott_baudrate;

    uint8_t reboot_character;               // which byte is used to reboot. Default 'R', could be changed carefully to something else.
} serialConfig_t;

uint8_t lookupScenarioIndex(serialPortFunctionScenario_e scenario);

serialPort_t *findOpenSerialPort(uint16_t functionMask);
serialPort_t *openSerialPort(serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, serialInversion_e inversion);

bool canOpenSerialPort(uint16_t functionMask);
void beginSerialPortFunction(serialPort_t *port, serialPortFunction_e function);
void endSerialPortFunction(serialPort_t *port, serialPortFunction_e function);
serialPortFunction_t *findSerialPortFunction(uint16_t functionMask);

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort);

void applySerialConfigToPortFunctions(serialConfig_t *serialConfig);
bool isSerialConfigValid(serialConfig_t *serialConfig);
bool doesConfigurationUsePort(serialConfig_t *serialConfig, serialPortIdentifier_e portIdentifier);

void evaluateOtherData(uint8_t sr);
void handleSerial(void);
