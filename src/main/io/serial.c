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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "cli/cli.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/system.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#if defined(USE_SOFTSERIAL)
#include "drivers/serial_softserial.h"
#endif

#if defined(SIMULATOR_BUILD)
#include "drivers/serial_tcp.h"
#endif

#include "drivers/light_led.h"

#if defined(USE_VCP)
#include "drivers/serial_usb_vcp.h"
#endif

#include "config/config.h"

#include "io/serial.h"

#include "msp/msp_serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "pg/pinio.h"

static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_UART0
    SERIAL_PORT_UART0,
#endif
#ifdef USE_UART1
    SERIAL_PORT_USART1,
#endif
#ifdef USE_UART2
    SERIAL_PORT_USART2,
#endif
#ifdef USE_UART3
    SERIAL_PORT_USART3,
#endif
#ifdef USE_UART4
    SERIAL_PORT_UART4,
#endif
#ifdef USE_UART5
    SERIAL_PORT_UART5,
#endif
#ifdef USE_UART6
    SERIAL_PORT_USART6,
#endif
#ifdef USE_UART7
    SERIAL_PORT_USART7,
#endif
#ifdef USE_UART8
    SERIAL_PORT_USART8,
#endif
#ifdef USE_UART9
    SERIAL_PORT_UART9,
#endif
#ifdef USE_UART10
    SERIAL_PORT_USART10,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
#ifdef USE_LPUART1
    SERIAL_PORT_LPUART1,
#endif
};

const char* serialPortNames[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    "VCP",
#endif
#ifdef USE_UART0
    "UART0",
#endif
#ifdef USE_UART1
    "UART1",
#endif
#ifdef USE_UART2
    "UART2",
#endif
#ifdef USE_UART3
    "UART3",
#endif
#ifdef USE_UART4
    "UART4",
#endif
#ifdef USE_UART5
    "UART5",
#endif
#ifdef USE_UART6
    "UART6",
#endif
#ifdef USE_UART7
    "UART7",
#endif
#ifdef USE_UART8
    "UART8",
#endif
#ifdef USE_UART9
    "UART9",
#endif
#ifdef USE_UART10
    "UART10",
#endif
#ifdef USE_SOFTSERIAL1
    "SOFT1",
#endif
#ifdef USE_SOFTSERIAL2
    "SOFT2",
#endif
#ifdef USE_LPUART1
    "LPUART1",
#endif
};

const uint32_t baudRates[BAUD_COUNT] = {
    0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
    400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000
}; // see baudRate_e

static serialPortConfig_t* findInPortConfigs_identifier(const serialPortConfig_t cfgs[], size_t count, serialPortIdentifier_e identifier)
{
    if (identifier == SERIAL_PORT_NONE || identifier == SERIAL_PORT_ALL) {
        return NULL;
    }

    for (unsigned i = 0; i < count; i++) {
        if (cfgs[i].identifier == identifier) {
            // drop const on return - wrapper function will add it back if necessary
            return (serialPortConfig_t*)&cfgs[i];
        }
    }

    return NULL;
}

serialPortConfig_t* serialFindPortConfigurationMutable(serialPortIdentifier_e identifier)
{
    return findInPortConfigs_identifier(serialConfigMutable()->portConfigs, ARRAYLEN(serialConfigMutable()->portConfigs), identifier);
}

const serialPortConfig_t* serialFindPortConfiguration(serialPortIdentifier_e identifier)
{
    return findInPortConfigs_identifier(serialConfig()->portConfigs, ARRAYLEN(serialConfig()->portConfigs), identifier);
}

PG_REGISTER_WITH_RESET_FN(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 1);

void pgResetFn_serialConfig(serialConfig_t *serialConfig)
{
    memset(serialConfig, 0, sizeof(serialConfig_t));

    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        serialPortConfig_t* pCfg = &serialConfig->portConfigs[i];
        pCfg->identifier = serialPortIdentifiers[i];
        pCfg->msp_baudrateIndex = BAUD_115200;
        pCfg->gps_baudrateIndex = BAUD_57600;
        pCfg->telemetry_baudrateIndex = BAUD_AUTO;
        pCfg->blackbox_baudrateIndex = BAUD_115200;
    }

    serialConfig->portConfigs[0].functionMask = FUNCTION_MSP;

#ifdef MSP_UART
    serialPortConfig_t *uart2Config = serialFindPortConfigurationMutable(MSP_UART);
    if (uart2Config) {
        uart2Config->functionMask = FUNCTION_MSP;
    }
#endif

#if defined(USE_GPS) && defined(GPS_UART)
    serialPortConfig_t *gpsUartConfig = serialFindPortConfigurationMutable(GPS_UART);
    if (gpsUartConfig) {
        gpsUartConfig->functionMask = FUNCTION_GPS;
    }
#endif

#ifdef SERIALRX_UART
    serialPortConfig_t *serialRxUartConfig = serialFindPortConfigurationMutable(SERIALRX_UART);
    if (serialRxUartConfig) {
        serialRxUartConfig->functionMask = FUNCTION_RX_SERIAL;
    }
#endif

#ifdef SBUS_TELEMETRY_UART
    serialPortConfig_t *serialTelemetryUartConfig = serialFindPortConfigurationMutable(SBUS_TELEMETRY_UART);
    if (serialTelemetryUartConfig) {
        serialTelemetryUartConfig->functionMask = FUNCTION_TELEMETRY_SMARTPORT;
    }
#endif

#ifdef ESC_SENSOR_UART
    serialPortConfig_t *escSensorUartConfig = serialFindPortConfigurationMutable(ESC_SENSOR_UART);
    if (escSensorUartConfig) {
        escSensorUartConfig->functionMask = FUNCTION_ESC_SENSOR;
    }
#endif

#ifdef USE_VTX
#ifdef VTX_SMARTAUDIO_UART
    serialPortConfig_t *vtxSmartAudioUartConfig = serialFindPortConfigurationMutable(VTX_SMARTAUDIO_UART);
    if (vtxSmartAudioUartConfig) {
        vtxSmartAudioUartConfig->functionMask = FUNCTION_VTX_SMARTAUDIO;
    }
#endif

#ifdef VTX_TRAMP_UART
    serialPortConfig_t *vtxTrampUartConfig = serialFindPortConfigurationMutable(VTX_TRAMP_UART);
    if (vtxTrampUartConfig) {
        vtxTrampUartConfig->functionMask = FUNCTION_VTX_TRAMP;
    }
#endif

#ifdef VTX_MSP_UART
    serialPortConfig_t *vtxMspUartConfig = serialFindPortConfigurationMutable(VTX_MSP_UART);
    if (vtxMspUartConfig) {
        vtxMspUartConfig->functionMask = FUNCTION_VTX_MSP;
    }
#endif
#endif // USE_VTX

#ifdef MSP_DISPLAYPORT_UART
    serialPortConfig_t *displayPortUartConfig = serialFindPortConfigurationMutable(MSP_DISPLAYPORT_UART);
    if (displayPortUartConfig) {
        displayPortUartConfig->functionMask = FUNCTION_VTX_MSP | FUNCTION_MSP;
    }
#endif

#if defined(USE_MSP_UART)
    serialPortConfig_t * uart1Config = serialFindPortConfigurationMutable(USE_MSP_UART);
    if (uart1Config) {
        uart1Config->functionMask = FUNCTION_MSP;
    }
#endif

    serialConfig->reboot_character = 'R';
    serialConfig->serial_update_rate_hz = 100;
}

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    for (unsigned index = 0; index < ARRAYLEN(baudRates); index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}

int findSerialPortIndexByIdentifier(serialPortIdentifier_e identifier)
{
    for (unsigned index = 0; index < ARRAYLEN(serialPortIdentifiers); index++) {
        if (serialPortIdentifiers[index] == identifier) {
            return index;
        }
    }
    return -1;
}

// find serial port by name.
// when cmp is NULL, case-insensitive compare is used
// cmp is strcmp-like function
serialPortIdentifier_e findSerialPortByName(const char* portName, int (*cmp)(const char *portName, const char *candidate))
{
    if (!cmp) { // use strcasecmp by default
        cmp = strcasecmp;
    }
    for (unsigned i = 0; i < ARRAYLEN(serialPortNames); i++) {
        if (cmp(portName, serialPortNames[i]) == 0) {
            return serialPortIdentifiers[i];  // 1:1 map between names and identifiers
        }
    }
    return SERIAL_PORT_NONE;
}

const char* serialName(serialPortIdentifier_e identifier, const char* notFound)
{
    const int idx = findSerialPortIndexByIdentifier(identifier);
    return idx >= 0 ? serialPortNames[idx] : notFound;
}

serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier)
{
    for (serialPortUsage_t* usage = serialPortUsageList; usage < ARRAYEND(serialPortUsageList); usage++) {
        if (usage->identifier == identifier) {
            return usage;
        }
    }
    return NULL;
}

static serialPortUsage_t *findSerialPortUsageByPort(const serialPort_t *serialPort)
{
    for (serialPortUsage_t* usage = serialPortUsageList; usage < ARRAYEND(serialPortUsageList); usage++) {
        if (usage->serialPort == serialPort) {
            return usage;
        }
    }
    return NULL;
}

typedef struct findSerialPortConfigState_s {
    uint8_t lastIndex;
} findSerialPortConfigState_t;

static findSerialPortConfigState_t findSerialPortConfigState;

const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    memset(&findSerialPortConfigState, 0, sizeof(findSerialPortConfigState));

    return findNextSerialPortConfig(function);
}

const serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
{
    while (findSerialPortConfigState.lastIndex < ARRAYLEN(serialConfig()->portConfigs)) {
        const serialPortConfig_t *candidate = &serialConfig()->portConfigs[findSerialPortConfigState.lastIndex++];

        if (candidate->functionMask & function) {
            return candidate;
        }
    }
    return NULL;
}

portSharing_e determinePortSharing(const serialPortConfig_t *portConfig, serialPortFunction_e function)
{
    if (!portConfig || (portConfig->functionMask & function) == 0) {
        return PORTSHARING_UNUSED;
    }
    return portConfig->functionMask == function ? PORTSHARING_NOT_SHARED : PORTSHARING_SHARED;
}

bool isSerialPortShared(const serialPortConfig_t *portConfig, uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    return (portConfig) && (portConfig->functionMask & sharedWithFunction) && (portConfig->functionMask & functionMask);
}

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    for (const serialPortConfig_t *candidate = serialConfig()->portConfigs;
         candidate < ARRAYEND(serialConfig()->portConfigs);
         candidate++) {
        if (isSerialPortShared(candidate, functionMask, sharedWithFunction)) {
            const serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(candidate->identifier);
            if (!serialPortUsage) {
                continue;
            }
            return serialPortUsage->serialPort;
        }
    }
    return NULL;
}

#ifdef USE_TELEMETRY
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX | TELEMETRY_PORT_FUNCTIONS_MASK | FUNCTION_VTX_MSP)
#else
#define ALL_FUNCTIONS_SHARABLE_WITH_MSP (FUNCTION_BLACKBOX | FUNCTION_VTX_MSP)
#endif

bool isSerialConfigValid(serialConfig_t *serialConfigToCheck)
{
    /*
     * rules:
     * - 1 MSP port minimum, max MSP ports is defined and must be adhered to.
     * - MSP is allowed to be shared with EITHER any telemetry OR blackbox.
     *   (using either / or, switching based on armed / disarmed or the AUX channel configured for BOXTELEMETRY)
     * - serial RX and FrSky / LTM / MAVLink telemetry can be shared
     *   (serial RX using RX line, telemetry using TX line)
     * - No other sharing combinations are valid.
     */
    uint8_t mspPortCount = 0;

    for (unsigned index = 0; index < ARRAYLEN(serialConfigToCheck->portConfigs); index++) {
        const serialPortConfig_t *portConfig = &serialConfigToCheck->portConfigs[index];

#ifdef USE_SOFTSERIAL
        if (serialType(portConfig->identifier) == SERIALTYPE_SOFTSERIAL) {
            // Ensure MSP or serial RX is not enabled on soft serial ports
            serialConfigToCheck->portConfigs[index].functionMask &= ~(FUNCTION_MSP | FUNCTION_RX_SERIAL);
            // Ensure that the baud rate on soft serial ports is limited to 19200
#ifndef USE_OVERRIDE_SOFTSERIAL_BAUDRATE
            serialConfigToCheck->portConfigs[index].gps_baudrateIndex = constrain(portConfig->gps_baudrateIndex, BAUD_AUTO, BAUD_19200);
            serialConfigToCheck->portConfigs[index].blackbox_baudrateIndex = constrain(portConfig->blackbox_baudrateIndex, BAUD_AUTO, BAUD_19200);
            serialConfigToCheck->portConfigs[index].telemetry_baudrateIndex = constrain(portConfig->telemetry_baudrateIndex, BAUD_AUTO, BAUD_19200);
#endif
        }
#endif // USE_SOFTSERIAL

        if (portConfig->functionMask & FUNCTION_MSP) {
            mspPortCount++;
        }
        if (portConfig->identifier == SERIAL_PORT_USB_VCP
            && (portConfig->functionMask & FUNCTION_MSP) == 0) {
            // Require MSP to be enabled for the VCP port
            return false;
        }

        uint8_t bitCount = popcount32(portConfig->functionMask);

#ifdef USE_VTX_MSP
        if ((portConfig->functionMask & FUNCTION_VTX_MSP) && bitCount == 1) { // VTX MSP has to be shared with RX or MSP serial
            return false;
        }
#endif

        if (bitCount > 1) {
            // shared
            if (bitCount > (BITCOUNT(FUNCTION_MSP | ALL_FUNCTIONS_SHARABLE_WITH_MSP))) {
                return false;
            }

            if ((portConfig->functionMask & FUNCTION_MSP) && (portConfig->functionMask & ALL_FUNCTIONS_SHARABLE_WITH_MSP)) {
                // MSP & telemetry
#ifdef USE_TELEMETRY
            } else if (telemetryCheckRxPortShared(portConfig, rxConfig()->serialrx_provider)) {
                // serial RX & telemetry
#endif
            } else {
                // some other combination
                return false;
            }
        }
    }

    if (mspPortCount == 0 || mspPortCount > MAX_MSP_PORT_COUNT) {
        return false;
    }
    return true;
}

bool doesConfigurationUsePort(serialPortIdentifier_e identifier)
{
    const serialPortConfig_t *candidate = serialFindPortConfiguration(identifier);
    return candidate != NULL && candidate->functionMask;
}

serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr rxCallback,
    void *rxCallbackData,
    uint32_t baudRate,
    portMode_e mode,
    portOptions_e options)
{
#if !(defined(USE_UART) || defined(USE_SOFTSERIAL) || defined(USE_LPUART))
    UNUSED(rxCallback);
    UNUSED(rxCallbackData);
    UNUSED(baudRate);
    UNUSED(mode);
    UNUSED(options);
#endif

    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(identifier);
    if (!serialPortUsage || serialPortUsage->function != FUNCTION_NONE) {
        // not available / already in use
        return NULL;
    }

    serialPort_t *serialPort = NULL;

    switch (serialType(identifier)) {
#if defined(USE_VCP)
    case SERIALTYPE_USB_VCP:
        serialPort = usbVcpOpen();
        break;
#endif
#if defined(USE_UART)
    case SERIALTYPE_UART:
    case SERIALTYPE_LPUART:
#if defined(SIMULATOR_BUILD)
        // emulate serial ports over TCP
        serialPort = serTcpOpen(identifier, rxCallback, rxCallbackData, baudRate, mode, options);
#else
        serialPort = uartOpen(identifier, rxCallback, rxCallbackData, baudRate, mode, options);
#endif
#endif
        break;
#ifdef USE_SOFTSERIAL
    case SERIALTYPE_SOFTSERIAL:
# if !defined(USE_OVERRIDE_SOFTSERIAL_BAUDRATE)
        if (baudRate > 19200) {
            // Don't continue if baud rate requested is higher then the limit set on soft serial ports
            return NULL;
        }
# endif // !USE_OVERRIDE_SOFTSERIAL_BAUDRATE
        serialPort = softSerialOpen(identifier, rxCallback, rxCallbackData, baudRate, mode, options);
        break;
#endif // USE_SOFTSERIAL
        default:
            break;
    }

    if (!serialPort) {
        return NULL;
    }

    serialPort->identifier = identifier; // Some versions of *Open() set this member sooner

    serialPortUsage->function = function;
    serialPortUsage->serialPort = serialPort;

    return serialPort;
}

void closeSerialPort(serialPort_t *serialPort)
{
    if (!serialPort) {
        return;
    }
    serialPortUsage_t *serialPortUsage = findSerialPortUsageByPort(serialPort);
    if (!serialPortUsage) {
        // already closed
        return;
    }

    // TODO wait until data has been transmitted.
    serialPort->rxCallback = NULL;

    serialPortUsage->function = FUNCTION_NONE;
    serialPortUsage->serialPort = NULL;
}

void serialInit(bool softserialEnabled)
{
    memset(&serialPortUsageList, 0, sizeof(serialPortUsageList));

    for (int index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsageList[index].identifier = serialPortIdentifiers[index];

#if SERIAL_TRAIT_PIN_CONFIG
        const int resourceIndex = serialResourceIndex(serialPortUsageList[index].identifier);
        if (resourceIndex >= 0 && !(serialPinConfig()->ioTagTx[resourceIndex] || serialPinConfig()->ioTagRx[resourceIndex])) {
            // resource exists but no pin is assigned
            serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
            continue;
        }
#endif

        if (serialType(serialPortUsageList[index].identifier) == SERIALTYPE_SOFTSERIAL && !softserialEnabled) {
            // soft serial is not enabled, or not built into the firmware
            serialPortUsageList[index].identifier = SERIAL_PORT_NONE;
            continue;
        }
    }
}

void serialRemovePort(serialPortIdentifier_e identifier)
{
    serialPortUsage_t* usage;
    while ((usage = findSerialPortUsageByIdentifier(identifier)) != NULL) {
        usage->identifier = SERIAL_PORT_NONE;
    }
}

bool serialIsPortAvailable(serialPortIdentifier_e identifier)
{
    return findSerialPortUsageByIdentifier(identifier) != NULL;
}

void waitForSerialPortToFinishTransmitting(serialPort_t *serialPort)
{
    while (!isSerialTransmitBufferEmpty(serialPort)) {
        delay(10);
    };
}

#if defined(USE_GPS) || defined(USE_SERIAL_PASSTHROUGH)
// Default data consumer for serialPassThrough.
static void nopConsumer(uint8_t data)
{
    UNUSED(data);
}

/*
 A high-level serial passthrough implementation. Used by cli to start an
 arbitrary serial passthrough "proxy". Optional callbacks can be given to allow
 for specialized data processing.
 */
void serialPassthrough(serialPort_t *left, serialPort_t *right, serialConsumer *leftC, serialConsumer *rightC)
{
    waitForSerialPortToFinishTransmitting(left);
    waitForSerialPortToFinishTransmitting(right);

    if (!leftC)
        leftC = &nopConsumer;
    if (!rightC)
        rightC = &nopConsumer;

    LED0_OFF;
    LED1_OFF;

    // Either port might be open in a mode other than MODE_RXTX. We rely on
    // serialRxBytesWaiting() to do the right thing for a TX only port. No
    // special handling is necessary OR performed.
    while (1) {
        // TODO: maintain a timestamp of last data received. Use this to
        // implement a guard interval and check for `+++` as an escape sequence
        // to return to CLI command mode.
        // https://en.wikipedia.org/wiki/Escape_sequence#Modem_control
        if (serialRxBytesWaiting(left)) {
            LED0_ON;
            uint8_t c = serialRead(left);
            // Make sure there is space in the tx buffer
            while (!serialTxBytesFree(right));
            serialWrite(right, c);
            leftC(c);
            LED0_OFF;
         }
         if (serialRxBytesWaiting(right)) {
             LED0_ON;
             uint8_t c = serialRead(right);
             // Make sure there is space in the tx buffer
             while (!serialTxBytesFree(left));
             serialWrite(left, c);
             rightC(c);
             LED0_OFF;
         }
     }
 }
 #endif
