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
#include "io/serial.h"
#include "io/serial_msp.h"

#include "telemetry/telemetry.h"
#include "telemetry/msp.h"

static telemetryConfig_t *telemetryConfig;
static serialPortConfig_t *portConfig;

static bool mspTelemetryEnabled =  false;
static portSharing_e mspPortSharing;

#define TELEMETRY_MSP_INITIAL_PORT_MODE MODE_TX

static serialPort_t *mspTelemetryPort = NULL;

void initMSPTelemetry(telemetryConfig_t *initialTelemetryConfig)
{
    telemetryConfig = initialTelemetryConfig;
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MSP);
    mspPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MSP);
}

void checkMSPTelemetryState(void)
{
    bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mspPortSharing);

    if (newTelemetryEnabledValue == mspTelemetryEnabled) {
        return;
    }

    if (newTelemetryEnabledValue)
        configureMSPTelemetryPort();
    else
        freeMSPTelemetryPort();
}

void handleMSPTelemetry(void)
{
    if (!mspTelemetryEnabled) {
        return;
    }

    if (!mspTelemetryPort) {
        return;
    }

    sendMspTelemetry();
}

void freeMSPTelemetryPort(void)
{
    mspReleasePortIfAllocated(mspTelemetryPort);
    closeSerialPort(mspTelemetryPort);
    mspTelemetryPort = NULL;
    mspTelemetryEnabled = false;
}

void configureMSPTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        baudRateIndex = BAUD_19200;
    }

    mspTelemetryPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MSP, NULL, baudRates[baudRateIndex], TELEMETRY_MSP_INITIAL_PORT_MODE, SERIAL_NOT_INVERTED);

    if (!mspTelemetryPort) {
        return;
    }
    mspSetTelemetryPort(mspTelemetryPort);

    mspTelemetryEnabled = true;
}

#endif
