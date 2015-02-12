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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_usb_vcp.h"

#include "io/serial.h"
#include "serial_cli.h"
#include "serial_msp.h"

#include "config/config.h"

#ifdef TELEMETRY
#include "telemetry/telemetry.h"
#endif

static serialConfig_t *serialConfig;
static serialPortUsage_t serialPortUsageList[SERIAL_PORT_COUNT];

serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
#ifdef USE_VCP
    SERIAL_PORT_USB_VCP,
#endif
#ifdef USE_USART1
    SERIAL_PORT_USART1,
#endif
#ifdef USE_USART2
    SERIAL_PORT_USART2,
#endif
#ifdef USE_USART3
    SERIAL_PORT_USART3,
#endif
#ifdef USE_SOFTSERIAL1
    SERIAL_PORT_SOFTSERIAL1,
#endif
#ifdef USE_SOFTSERIAL2
    SERIAL_PORT_SOFTSERIAL2,
#endif
};

static serialPortUsage_t *findSerialPortUsageByIdentifier(serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &serialPortUsageList[index];
        if (candidate->identifier == identifier) {
            return candidate;
        }
    }
    return NULL;
}

serialPortUsage_t *findSerialPortUsageByPort(serialPort_t *serialPort) {
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsage_t *candidate = &serialPortUsageList[index];
        if (candidate->serialPort == serialPort) {
            return candidate;
        }
    }
    return NULL;
}

typedef struct findSerialPortConfigState_s {
    uint8_t lastIndex;
} findSerialPortConfigState_t;

static findSerialPortConfigState_t findSerialPortConfigState;

serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
{
    memset(&findSerialPortConfigState, 0, sizeof(findSerialPortConfigState));

    return findNextSerialPortConfig(function);
}

serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
{
    while (findSerialPortConfigState.lastIndex < SERIAL_PORT_COUNT) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[findSerialPortConfigState.lastIndex++];

        if (candidate->functionMask & function) {
            return candidate;
        }
    }
    return NULL;
}

typedef struct findSharedSerialPortState_s {
    uint8_t lastIndex;
} findSharedSerialPortState_t;

static findSharedSerialPortState_t findSharedSerialPortState;

serialPort_t *findSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    memset(&findSharedSerialPortState, 0, sizeof(findSharedSerialPortState));

    return findNextSharedSerialPort(functionMask, sharedWithFunction);
}

serialPort_t *findNextSharedSerialPort(uint16_t functionMask, serialPortFunction_e sharedWithFunction)
{
    while (findSharedSerialPortState.lastIndex < SERIAL_PORT_COUNT) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[findSharedSerialPortState.lastIndex++];

        if ((candidate->functionMask & sharedWithFunction) && (candidate->functionMask & functionMask)) {
            serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(candidate->identifier);
            if (!serialPortUsage) {
                continue;
            }
            return serialPortUsage->serialPort;
        }
    }
    return NULL;
}

bool isSerialConfigValid(serialConfig_t *serialConfigToCheck)
{
    UNUSED(serialConfigToCheck);
    return true; // FIXME implement rules - 1 MSP port minimum.
}

bool doesConfigurationUsePort(serialPortIdentifier_e identifier)
{
    uint8_t index;
    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortConfig_t *candidate = &serialConfig->portConfigs[index];
        if (candidate->identifier == identifier && candidate->functionMask) {
            return true;
        }
    }
    return false;
}

serialPort_t *openSerialPort(
    serialPortIdentifier_e identifier,
    serialPortFunction_e function,
    serialReceiveCallbackPtr callback,
    uint32_t baudRate,
    portMode_t mode,
    serialInversion_e inversion)
{
    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(identifier);
    if (serialPortUsage->function != FUNCTION_NONE) {
        // already in use
        return NULL;
    }

    serialPort_t *serialPort = NULL;

    switch(identifier) {
#ifdef USE_VCP
        case SERIAL_PORT_USB_VCP:
            serialPort = usbVcpOpen();
            break;
#endif
#ifdef USE_USART1
        case SERIAL_PORT_USART1:
            serialPort = uartOpen(USART1, callback, baudRate, mode, inversion);
            break;
#endif
#ifdef USE_USART2
        case SERIAL_PORT_USART2:
            serialPort = uartOpen(USART2, callback, baudRate, mode, inversion);
            break;
#endif
#ifdef USE_USART3
        case SERIAL_PORT_USART3:
            serialPort = uartOpen(USART3, callback, baudRate, mode, inversion);
            break;
#endif
#ifdef USE_SOFTSERIAL1
        case SERIAL_PORT_SOFTSERIAL1:
            serialPort = openSoftSerial(SOFTSERIAL1, callback, baudRate, inversion);
            serialSetMode(serialPort, mode);
            break;
#endif
#ifdef USE_SOFTSERIAL2
        case SERIAL_PORT_SOFTSERIAL2:
            serialPort = openSoftSerial(SOFTSERIAL2, callback, baudRate, inversion);
            serialSetMode(serialPort, mode);
            break;
#endif
        default:
            break;
    }

    if (!serialPort) {
        return NULL;
    }

    serialPort->identifier = identifier;

    serialPortUsage->function = function;
    serialPortUsage->serialPort = serialPort;

    return serialPort;
}

void closeSerialPort(serialPort_t *serialPort) {
    serialPortUsage_t *serialPortUsage = findSerialPortUsageByPort(serialPort);
    if (!serialPortUsage) {
        // already closed
        return;
    }

    serialPort->callback = NULL;

    serialPortUsage->function = FUNCTION_NONE;
    serialPortUsage->serialPort = NULL;
}

void serialInit(serialConfig_t *initialSerialConfig)
{
    uint8_t index;

    serialConfig = initialSerialConfig;

    memset(&serialPortUsageList, 0, sizeof(serialPortUsageList));

    for (index = 0; index < SERIAL_PORT_COUNT; index++) {
        serialPortUsageList[index].identifier = serialPortIdentifiers[index];
    }
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

void cliEnter(serialPort_t *serialPort);

void evaluateOtherData(serialPort_t *serialPort, uint8_t receivedChar)
{
    if (receivedChar == '#') {
        cliEnter(serialPort);
    } else if (receivedChar == serialConfig->reboot_character) {
        systemResetToBootloader();
    }
}

