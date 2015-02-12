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

#include "platform.h"

#ifdef TELEMETRY

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "io/serial.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "config/runtime_config.h"
#include "config/config.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/msp.h"
#include "telemetry/smartport.h"


static bool telemetryEnabled = false;

static telemetryConfig_t *telemetryConfig;

void useTelemetryConfig(telemetryConfig_t *telemetryConfigToUse)
{
    telemetryConfig = telemetryConfigToUse;
}

void telemetryInit()
{
    initFrSkyTelemetry(telemetryConfig);

    initHoTTTelemetry(telemetryConfig);

    initMSPTelemetry(telemetryConfig);

    initSmartPortTelemetry(telemetryConfig);

    checkTelemetryState();
}

bool determineNewTelemetryEnabledState(void)
{
    bool enabled = true;

    if (telemetryConfig->telemetry_switch)
        enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
    else
        enabled = ARMING_FLAG(ARMED);

    return enabled;
}

bool shouldChangeTelemetryStateNow(bool newState)
{
    return newState != telemetryEnabled;
}

static void configureTelemetryPort(void)
{
    configureFrSkyTelemetryPort();
    configureHoTTTelemetryPort();
    configureMSPTelemetryPort();
    configureSmartPortTelemetryPort();
}

void freeTelemetryPort(void)
{
    freeFrSkyTelemetryPort();
    freeHoTTTelemetryPort();
    freeMSPTelemetryPort();
    freeSmartPortTelemetryPort();
}

void checkTelemetryState(void)
{
    bool newEnabledState = determineNewTelemetryEnabledState();

    if (!shouldChangeTelemetryStateNow(newEnabledState)) {
        return;
    }

    if (newEnabledState)
        configureTelemetryPort();
    else
        freeTelemetryPort();

    telemetryEnabled = newEnabledState;
}

void handleTelemetry(void)
{
    if (!determineNewTelemetryEnabledState())
        return;

    if (!telemetryEnabled) {
        return;
    }

    handleFrSkyTelemetry();
    handleHoTTTelemetry();
    handleMSPTelemetry();
    handleSmartPortTelemetry();
}

#endif
