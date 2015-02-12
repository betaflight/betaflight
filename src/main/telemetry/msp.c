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

/*
 * telemetry_MSP.c
 *
 *  Created on: 22 Apr 2014
 *      Author: trey marc
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build_config.h"

#ifdef TELEMETRY

#include "drivers/serial.h"
#include "telemetry/telemetry.h"
#include "io/serial.h"
#include "io/serial_msp.h"

static telemetryConfig_t *telemetryConfig;

#define MSP_TELEMETRY_BAUDRATE 19200 // TODO make this configurable
#define MSP_TELEMETRY_INITIAL_PORT_MODE MODE_TX

static serialPort_t *mspTelemetryPort = NULL;

void initMSPTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
}

void handleMSPTelemetry(void)
{
    if (!mspTelemetryPort) {
        return;
    }

    sendMspTelemetry();
}

void freeMSPTelemetryPort(void)
{
    closeSerialPort(mspTelemetryPort);
    mspTelemetryPort = NULL;
}

void configureMSPTelemetryPort(void)
{
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP_TELEMETRY);
    if (!portConfig) {
        return;
    }

    mspTelemetryPort = openSerialPort(portConfig->identifier, FUNCTION_MSP_TELEMETRY, NULL, MSP_TELEMETRY_BAUDRATE, MSP_TELEMETRY_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!mspTelemetryPort) {
        return;
    }
    mspSetTelemetryPort(mspTelemetryPort);
}

#endif
