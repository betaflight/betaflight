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

#include "common/utils.h"

#include "drivers/serial.h"

#include "io/serial.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "config/config.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/ibus.h"

static telemetryConfig_t *telemetryConfig;

void telemetryUseConfig(telemetryConfig_t *telemetryConfigToUse)
{
    telemetryConfig = telemetryConfigToUse;
}

void telemetryInit(void)
{
#if defined(TELEMETRY_FRSKY)
    initFrSkyTelemetry(telemetryConfig);
#endif

#if defined(TELEMETRY_HOTT)
    initHoTTTelemetry(telemetryConfig);
#endif

#if defined(TELEMETRY_SMARTPORT)
    initSmartPortTelemetry(telemetryConfig);
#endif

#if defined(TELEMETRY_LTM)
    initLtmTelemetry(telemetryConfig);
#endif

#if defined(TELEMETRY_MAVLINK)
    initMAVLinkTelemetry();
#endif

#if defined(TELEMETRY_JETIEXBUS)
    initJetiExBusTelemetry();
#endif

#if defined(TELEMETRY_IBUS)
    initIbusTelemetry(telemetryConfig);
#endif

    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (telemetryConfig->telemetry_switch)
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig)
{
    return portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK;
}

serialPort_t *telemetrySharedPort = NULL;

void telemetryCheckState(void)
{
#if defined(TELEMETRY_FRSKY)
    checkFrSkyTelemetryState();
#endif

#if defined(TELEMETRY_HOTT)
    checkHoTTTelemetryState();
#endif

#if defined(TELEMETRY_SMARTPORT)
    checkSmartPortTelemetryState();
#endif

#if defined(TELEMETRY_LTM)
    checkLtmTelemetryState();
#endif

#if defined(TELEMETRY_MAVLINK)
    checkMAVLinkTelemetryState();
#endif

#if defined(TELEMETRY_JETIEXBUS)
    checkJetiExBusTelemetryState();
#endif

#if defined(TELEMETRY_IBUS)
    checkIbusTelemetryState();
#endif

}

void telemetryProcess(timeUs_t currentTimeUs, rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
#if defined(TELEMETRY_FRSKY)
    handleFrSkyTelemetry(rxConfig, deadband3d_throttle);
#else
    UNUSED(rxConfig);
    UNUSED(deadband3d_throttle);
#endif

#if defined(TELEMETRY_HOTT)
    handleHoTTTelemetry(currentTimeUs);
#else
    UNUSED(currentTimeUs);
#endif

#if defined(TELEMETRY_SMARTPORT)
    handleSmartPortTelemetry();
#endif

#if defined(TELEMETRY_LTM)
    handleLtmTelemetry();
#endif

#if defined(TELEMETRY_MAVLINK)
    handleMAVLinkTelemetry(currentTimeUs);
#else
    UNUSED(currentTimeUs);
#endif

#if defined(TELEMETRY_JETIEXBUS)
    handleJetiExBusTelemetry();
#endif

#if defined(TELEMETRY_IBUS)
    handleIbusTelemetry();
#endif

}

#endif
