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

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef TELEMETRY

#include "common/utils.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/mavlink.h"
#include "telemetry/crsf.h"

static telemetryConfig_t *telemetryConfig;

void telemetryUseConfig(telemetryConfig_t *telemetryConfigToUse)
{
    telemetryConfig = telemetryConfigToUse;
}

void telemetryInit(void)
{
#ifdef TELEMETRY_FRSKY
    initFrSkyTelemetry(telemetryConfig);
#endif
#ifdef TELEMETRY_HOTT
    initHoTTTelemetry(telemetryConfig);
#endif
#ifdef TELEMETRY_SMARTPORT
    initSmartPortTelemetry(telemetryConfig);
#endif
#ifdef TELEMETRY_LTM
    initLtmTelemetry(telemetryConfig);
#endif
#ifdef TELEMETRY_JETIEXBUS
    initJetiExBusTelemetry(telemetryConfig);
#endif
#ifdef TELEMETRY_MAVLINK
    initMAVLinkTelemetry();
#endif
#ifdef TELEMETRY_CRSF
    initCrsfTelemetry();
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
#ifdef TELEMETRY_FRSKY
    checkFrSkyTelemetryState();
#endif
#ifdef TELEMETRY_HOTT
    checkHoTTTelemetryState();
#endif
#ifdef TELEMETRY_SMARTPORT
    checkSmartPortTelemetryState();
#endif
#ifdef TELEMETRY_LTM
    checkLtmTelemetryState();
#endif
#ifdef TELEMETRY_JETIEXBUS
    checkJetiExBusTelemetryState();
#endif
#ifdef TELEMETRY_MAVLINK
    checkMAVLinkTelemetryState();
#endif
#ifdef TELEMETRY_CRSF
    checkCrsfTelemetryState();
#endif
}

void telemetryProcess(uint32_t currentTime, rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
#ifdef TELEMETRY_FRSKY
    handleFrSkyTelemetry(rxConfig, deadband3d_throttle);
#else
    UNUSED(rxConfig);
    UNUSED(deadband3d_throttle);
#endif
#ifdef TELEMETRY_HOTT
    handleHoTTTelemetry(currentTime);
#else
    UNUSED(currentTime);
#endif
#ifdef TELEMETRY_SMARTPORT
    handleSmartPortTelemetry();
#endif
#ifdef TELEMETRY_LTM
    handleLtmTelemetry();
#endif
#ifdef TELEMETRY_JETIEXBUS
    handleJetiExBusTelemetry();
#endif
#ifdef TELEMETRY_MAVLINK
    handleMAVLinkTelemetry();
#endif
#ifdef TELEMETRY_CRSF
    handleCrsfTelemetry(currentTime);
#endif
}

#endif
