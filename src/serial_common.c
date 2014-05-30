#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "drivers/system_common.h"
#include "drivers/gpio_common.h"
#include "drivers/timer_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart_common.h"

#include "serial_cli.h"
#include "serial_msp.h"

#include "serial_common.h"
#include "config.h"

uint32_t getTelemetryProviderBaudRate(void);
void updateSerialRxFunctionConstraint(functionConstraint_t *functionConstraintToUpdate);

void mspInit(serialConfig_t *serialConfig);
void cliInit(serialConfig_t *serialConfig);

// this exists so the user can reference scenarios by a number in the CLI instead of an unuser-friendly bitmask.
const serialPortFunctionScenario_e serialPortScenarios[SERIAL_PORT_SCENARIO_COUNT] = {
    SCENARIO_UNUSED,

    // common scenarios in order of importance
    SCENARIO_MSP_CLI_TELEMETRY_GPS_PASTHROUGH,
    SCENARIO_GPS_ONLY,
    SCENARIO_SERIAL_RX_ONLY,
    SCENARIO_TELEMETRY_ONLY,

    // other scenarios
    SCENARIO_MSP_CLI_GPS_PASTHROUGH,
    SCENARIO_CLI_ONLY,
    SCENARIO_GPS_PASSTHROUGH_ONLY,
    SCENARIO_MSP_ONLY
};

static serialConfig_t *serialConfig;
static serialPort_t *serialPorts[SERIAL_PORT_COUNT];
static serialPortFunction_t serialPortFunctions[SERIAL_PORT_COUNT] = {
    {SERIAL_PORT_USART1,      NULL, SCENARIO_UNUSED, FUNCTION_NONE},
    {SERIAL_PORT_USART2,      NULL, SCENARIO_UNUSED, FUNCTION_NONE},
    {SERIAL_PORT_SOFTSERIAL1, NULL, SCENARIO_UNUSED, FUNCTION_NONE},
    {SERIAL_PORT_SOFTSERIAL2, NULL, SCENARIO_UNUSED, FUNCTION_NONE}
};

const static serialPortConstraint_t serialPortConstraints[SERIAL_PORT_COUNT] = {
    {SERIAL_PORT_USART1,        9600, 115200,   SPF_NONE | SPF_SUPPORTS_SBUS_MODE },
    {SERIAL_PORT_USART2,        9600, 115200,   SPF_SUPPORTS_CALLBACK | SPF_SUPPORTS_SBUS_MODE},
    {SERIAL_PORT_SOFTSERIAL1,   9600, 19200,    SPF_SUPPORTS_CALLBACK | SPF_IS_SOFTWARE_INVERTABLE},
    {SERIAL_PORT_SOFTSERIAL2,   9600, 19200,    SPF_SUPPORTS_CALLBACK | SPF_IS_SOFTWARE_INVERTABLE}
};

const functionConstraint_t functionConstraints[] = {
        { FUNCTION_CLI,             9600, 115200, NO_AUTOBAUD, SPF_NONE },
        { FUNCTION_GPS,             9600, 115200, AUTOBAUD,    SPF_NONE },
        { FUNCTION_GPS_PASSTHROUGH, 9600, 115200, NO_AUTOBAUD, SPF_NONE },
        { FUNCTION_MSP,             9600, 115200, NO_AUTOBAUD, SPF_NONE },
        { FUNCTION_SERIAL_RX,       9600, 115200, NO_AUTOBAUD, SPF_SUPPORTS_SBUS_MODE | SPF_SUPPORTS_CALLBACK },
        { FUNCTION_TELEMETRY,       9600, 19200,  NO_AUTOBAUD, SPF_NONE }
};

#define FUNCTION_CONSTRAINT_COUNT (sizeof(functionConstraints) / sizeof(functionConstraint_t))

typedef struct serialPortSearchResult_s {
    serialPortIndex_e portIndex;
    serialPortFunction_t *portFunction;
    const serialPortConstraint_t *portConstraint;
    const functionConstraint_t *functionConstraint;

    // private
    uint8_t startSerialPortFunctionIndex; // used by findNextSerialPort
} serialPortSearchResult_t;

static const serialPortFunctionList_t serialPortFunctionList = {
        4,
        serialPortFunctions
};

const serialPortFunctionList_t *getSerialPortFunctionList(void)
{
    return &serialPortFunctionList;
}

uint8_t lookupScenarioIndex(serialPortFunctionScenario_e scenario) {
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_SCENARIO_COUNT; index++) {
        if (serialPortScenarios[index] == scenario) {
            break;
        }
    }
    return index;
}

static serialPortIndex_e lookupSerialPortIndexByIdentifier(serialPortIdentifier_e identifier)
{
    serialPortIndex_e portIndex;
    for (portIndex = 0; portIndex < SERIAL_PORT_COUNT; portIndex++) {
        if (serialPortConstraints[portIndex].identifier == identifier) {
            break;
        }
    }
    return portIndex;
}

static uint8_t lookupSerialPortFunctionIndexByIdentifier(serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        if (serialPortFunctions[index].identifier == identifier) {
            break;
        }
    }
    return index;
}

static const functionConstraint_t *findFunctionConstraint(serialPortFunction_e function)
{
    const functionConstraint_t *functionConstraint = NULL;
    uint8_t functionConstraintIndex;

    for (functionConstraintIndex = 0; functionConstraintIndex < FUNCTION_CONSTRAINT_COUNT; functionConstraintIndex++) {
        functionConstraint = &functionConstraints[functionConstraintIndex];
        if (functionConstraint->function == function) {
            return functionConstraint;
        }
    }
    return NULL;
}

static uint8_t countBits_uint32(uint32_t n) {
  uint8_t c; // c accumulates the total bits set in n
  for (c = 0; n; c++)
    n &= n - 1; // clear the least significant bit set
  return c;
}
static int serialPortFunctionMostSpecificFirstComparator(const void *aPtr, const void *bPtr)
{
    serialPortFunction_t *a = (serialPortFunction_t *)aPtr;
    serialPortFunction_t *b = (serialPortFunction_t *)bPtr;

    return countBits_uint32(a->scenario) > countBits_uint32(b->scenario);
}

static void sortSerialPortFunctions(serialPortFunction_t *serialPortFunctions, uint8_t elements)
{
    qsort(serialPortFunctions, elements, sizeof(serialPortFunction_t), serialPortFunctionMostSpecificFirstComparator);
}

serialPortSearchResult_t *findNextSerialPort(serialPortFunction_e function, const functionConstraint_t *functionConstraint, serialPortSearchResult_t *resultBuffer)
{
    uint8_t serialPortFunctionIndex;
    serialPortFunction_t *serialPortFunction;
    for (serialPortFunctionIndex = resultBuffer->startSerialPortFunctionIndex; serialPortFunctionIndex < SERIAL_PORT_COUNT; serialPortFunctionIndex++) {
        serialPortFunction = &serialPortFunctions[serialPortFunctionIndex];

        if (!(serialPortFunction->scenario & function)) {
            continue;
        }

        uint8_t serialPortIndex = lookupSerialPortIndexByIdentifier(serialPortFunction->identifier);
        const serialPortConstraint_t *serialPortConstraint = &serialPortConstraints[serialPortIndex];

        if (!feature(FEATURE_SOFTSERIAL) && (
                serialPortConstraint->identifier == SERIAL_PORT_SOFTSERIAL1 ||
                serialPortConstraint->identifier == SERIAL_PORT_SOFTSERIAL2
        )) {
            continue;
        }

        if (functionConstraint->requiredSerialPortFeatures != SPF_NONE) {
            if (!(serialPortConstraint->feature & functionConstraint->requiredSerialPortFeatures)) {
                continue;
            }
        }

        if (functionConstraint->minBaudRate < serialPortConstraint->minBaudRate || functionConstraint->maxBaudRate > serialPortConstraint->maxBaudRate) {
            continue;
        }

        resultBuffer->portIndex = serialPortIndex;
        resultBuffer->portConstraint = serialPortConstraint;
        resultBuffer->portFunction = serialPortFunction;
        resultBuffer->functionConstraint = functionConstraint;

        uint8_t nextStartIndex = serialPortFunctionIndex + 1;
        resultBuffer->startSerialPortFunctionIndex = nextStartIndex;

        return resultBuffer;
    }

    return NULL;
}

/*
 * since this method, and other methods that use it, use a single instance of
 * searchPortSearchResult be sure to copy the data out of it before it gets overwritten by another caller.
 * If this becomes a problem perhaps change the implementation to use a destination argument.
 */
static serialPortSearchResult_t *findSerialPort(serialPortFunction_e function, const functionConstraint_t *functionConstraint)
{
    static serialPortSearchResult_t serialPortSearchResult;

    memset(&serialPortSearchResult, 0, sizeof(serialPortSearchResult));

    // FIXME this only needs to be done once, after the config has been loaded.
    sortSerialPortFunctions(serialPortFunctions, SERIAL_PORT_COUNT);

    return findNextSerialPort(function, functionConstraint, &serialPortSearchResult);
}


static serialPortFunction_t *findSerialPortFunction(uint16_t functionMask)
{
    serialPortIndex_e portIndex;

    // find exact match first
    for (portIndex = 0; portIndex < SERIAL_PORT_COUNT; portIndex++) {
        serialPortFunction_t *serialPortFunction = &serialPortFunctions[portIndex];
        if (serialPortFunction->scenario == functionMask) {
            return serialPortFunction;
        }
    }

    // find the first port that supports the function requested
    for (portIndex = 0; portIndex < SERIAL_PORT_COUNT; portIndex++) {
        serialPortFunction_t *serialPortFunction = &serialPortFunctions[portIndex];
        if (serialPortFunction->scenario & functionMask) {
            return serialPortFunction;
        }
    }

    return NULL;
}

/*
 * find a serial port that is:
 * a) open
 * b) matches the function mask exactly, or if an exact match is not found the first port that supports the function
 */
serialPort_t *findOpenSerialPort(uint16_t functionMask)
{
    serialPortFunction_t *function = findSerialPortFunction(functionMask);
    if (!function) {
        return NULL;
    }
    return function->port;
}



static serialPortFunction_t * findSerialPortFunctionByPort(serialPort_t *port)
{
    serialPortFunction_t *serialPortFunction;
    uint8_t functionIndex;

    for (functionIndex = 0; functionIndex < SERIAL_PORT_COUNT; functionIndex++) {
        serialPortFunction = &serialPortFunctions[functionIndex];
        if (serialPortFunction->port == port) {
            return serialPortFunction;
        }
    }

    return NULL;
}

void beginSerialPortFunction(serialPort_t *port, serialPortFunction_e function)
{
    serialPortFunction_t *serialPortFunction = findSerialPortFunctionByPort(port);

    serialPortFunction->currentFunction = function;
}

void endSerialPortFunction(serialPort_t *port, serialPortFunction_e function)
{
    serialPortFunction_t *serialPortFunction = findSerialPortFunctionByPort(port);

    serialPortFunction->currentFunction = FUNCTION_NONE;
}

functionConstraint_t *getConfiguredFunctionConstraint(serialPortFunction_e function)
{
    static functionConstraint_t configuredFunctionConstraint;
    const functionConstraint_t *functionConstraint;

    functionConstraint = findFunctionConstraint(function);
    if (!functionConstraint) {
        return NULL;
    }
    memcpy(&configuredFunctionConstraint, functionConstraint, sizeof(functionConstraint_t));

    switch(function) {
        case FUNCTION_MSP:
            configuredFunctionConstraint.maxBaudRate = serialConfig->msp_baudrate;
            break;

        case FUNCTION_CLI:
            configuredFunctionConstraint.minBaudRate = serialConfig->cli_baudrate;
            configuredFunctionConstraint.maxBaudRate = configuredFunctionConstraint.minBaudRate;
            break;

        case FUNCTION_GPS_PASSTHROUGH:
            configuredFunctionConstraint.minBaudRate = serialConfig->gps_passthrough_baudrate;
            configuredFunctionConstraint.maxBaudRate = configuredFunctionConstraint.minBaudRate;
            break;

        case FUNCTION_TELEMETRY:
            configuredFunctionConstraint.minBaudRate = getTelemetryProviderBaudRate();
            configuredFunctionConstraint.maxBaudRate = configuredFunctionConstraint.minBaudRate;
            break;
        case FUNCTION_SERIAL_RX:
            updateSerialRxFunctionConstraint(&configuredFunctionConstraint);
            break;
        case FUNCTION_GPS:
            configuredFunctionConstraint.maxBaudRate = serialConfig->gps_baudrate;
            break;
        default:
            break;
    }
    return &configuredFunctionConstraint;
}

bool isSerialConfigValid(serialConfig_t *serialConfigToCheck)
{
    serialPortSearchResult_t *searchResult;
    functionConstraint_t *functionConstraint;

    serialConfig = serialConfigToCheck;

    functionConstraint = getConfiguredFunctionConstraint(FUNCTION_MSP);
    searchResult = findSerialPort(FUNCTION_MSP, functionConstraint);
    if (!searchResult) {
        return false;
    }

    functionConstraint = getConfiguredFunctionConstraint(FUNCTION_CLI);
    searchResult = findSerialPort(FUNCTION_CLI, functionConstraint);
    if (!searchResult) {
        return false;
    }

    functionConstraint = getConfiguredFunctionConstraint(FUNCTION_GPS);
    searchResult = findSerialPort(FUNCTION_GPS, functionConstraint);
    if (feature(FEATURE_GPS) && !searchResult) {
        return false;
    }

    functionConstraint = getConfiguredFunctionConstraint(FUNCTION_SERIAL_RX);
    searchResult = findSerialPort(FUNCTION_SERIAL_RX, functionConstraint);
    if (feature(FEATURE_RX_SERIAL) && !searchResult) {
        return false;
    }

    functionConstraint = getConfiguredFunctionConstraint(FUNCTION_TELEMETRY);
    searchResult = findSerialPort(FUNCTION_TELEMETRY, functionConstraint);
    if (feature(FEATURE_TELEMETRY) && !searchResult) {
        return false;
    }

    return true;
}

bool doesConfigurationUsePort(serialConfig_t *serialConfig, serialPortIdentifier_e portIdentifier)
{
    serialPortSearchResult_t *searchResult;
    const functionConstraint_t *functionConstraint;
    serialPortFunction_e function;

    uint8_t index;
    for (index = 0; index < FUNCTION_CONSTRAINT_COUNT; index++) {
        function = functionConstraints[index].function;
        functionConstraint = findFunctionConstraint(function);
        searchResult = findSerialPort(function, functionConstraint);
        if (searchResult->portConstraint->identifier == portIdentifier) {
            return true;
        }
    }
    return false;
}

bool canOpenSerialPort(serialPortFunction_e function)
{
    functionConstraint_t *functionConstraint = getConfiguredFunctionConstraint(function);

    serialPortSearchResult_t *result = findSerialPort(function, functionConstraint);
    return result != NULL;
}

bool isSerialPortFunctionShared(serialPortFunction_e functionToUse, uint16_t functionMask)
{
    functionConstraint_t *functionConstraint = getConfiguredFunctionConstraint(functionToUse);
    serialPortSearchResult_t *result = findSerialPort(functionToUse, functionConstraint);
    if (!result) {
        return false;
    }

    return result->portFunction->scenario & functionMask;
}

void applySerialConfigToPortFunctions(serialConfig_t *serialConfig)
{
    serialPortFunctions[lookupSerialPortFunctionIndexByIdentifier(SERIAL_PORT_USART1)].scenario = serialPortScenarios[serialConfig->serial_port_1_scenario];
    serialPortFunctions[lookupSerialPortFunctionIndexByIdentifier(SERIAL_PORT_USART2)].scenario = serialPortScenarios[serialConfig->serial_port_2_scenario];
    serialPortFunctions[lookupSerialPortFunctionIndexByIdentifier(SERIAL_PORT_SOFTSERIAL1)].scenario = serialPortScenarios[serialConfig->serial_port_3_scenario];
    serialPortFunctions[lookupSerialPortFunctionIndexByIdentifier(SERIAL_PORT_SOFTSERIAL2)].scenario = serialPortScenarios[serialConfig->serial_port_4_scenario];
}

serialPort_t *openSerialPort(serialPortFunction_e function, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, serialInversion_e inversion)
{
    serialPort_t *serialPort = NULL;

    functionConstraint_t *initialFunctionConstraint = getConfiguredFunctionConstraint(function);

    functionConstraint_t updatedFunctionConstraint;
    memcpy(&updatedFunctionConstraint, initialFunctionConstraint, sizeof(updatedFunctionConstraint));
    if (initialFunctionConstraint->autoBaud == NO_AUTOBAUD) {
        updatedFunctionConstraint.minBaudRate = baudRate;
        updatedFunctionConstraint.maxBaudRate = baudRate;
    }
    functionConstraint_t *functionConstraint = &updatedFunctionConstraint;

    serialPortSearchResult_t *searchResult = findSerialPort(function, functionConstraint);

    while(searchResult && searchResult->portFunction->port) {
        // port is already open, find the next one
        searchResult = findNextSerialPort(function, functionConstraint, searchResult);
    }

    if (!searchResult) {
        return NULL;
    }

    serialPortIndex_e portIndex = searchResult->portIndex;

    const serialPortConstraint_t *serialPortConstraint = searchResult->portConstraint;

    serialPortIdentifier_e identifier = serialPortConstraint->identifier;
    switch(identifier) {
        case SERIAL_PORT_USART1:
            serialPort = uartOpen(USART1, callback, baudRate, mode, inversion);
            break;
        case SERIAL_PORT_USART2:
            serialPort = uartOpen(USART2, callback, baudRate, mode, inversion);
            break;
        case SERIAL_PORT_SOFTSERIAL1:
            serialPort = openSoftSerial(SOFTSERIAL1, callback, baudRate, inversion);
            serialSetMode(serialPort, mode);
            break;
        case SERIAL_PORT_SOFTSERIAL2:
            serialPort = openSoftSerial(SOFTSERIAL2, callback, baudRate, inversion);
            serialSetMode(serialPort, mode);
            break;
    }

    if (!serialPort) {
        return NULL;
    }

    serialPort->identifier = identifier;

    serialPorts[portIndex] = serialPort;

    serialPortFunction_t *serialPortFunction = searchResult->portFunction;

    serialPortFunction->port = serialPort;
    //serialPortFunction->identifier = identifier;

    beginSerialPortFunction(serialPort, function);

    return serialPort;
}

void serialInit(serialConfig_t *initialSerialConfig)
{
    serialConfig = initialSerialConfig;
    applySerialConfigToPortFunctions(serialConfig);

    mspInit(serialConfig);
    cliInit(serialConfig);
}

void handleSerial(void)
{
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
        return;
    }

    mspProcess();
}

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort)
{
    while (!isSerialTransmitBufferEmpty(serialPort)) {
        delay(10);
    };
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr)
{
    if (sr == '#')
        cliProcess();
    else if (sr == serialConfig->reboot_character)
        systemReset(true);      // reboot to bootloader
}

