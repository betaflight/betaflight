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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/serial.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "io/serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/ibus.h"
#include "telemetry/crsf.h"

PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);

#if defined(STM32F303xC)
#define TELEMETRY_DEFAULT_INVERSION 1
#else
#define TELEMETRY_DEFAULT_INVERSION 0
#endif

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inversion = TELEMETRY_DEFAULT_INVERSION,
    .telemetry_switch = 0,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = FRSKY_UNIT_METRICS,
    .frsky_vfas_precision = 0,
    .frsky_vfas_cell_voltage = 0,
    .hottAlarmSoundInterval = 5,
    .smartportUartUnidirectional = 0,
    .smartportFuelPercent = 0,
    .ibusTelemetryType = 0,
    .ltmUpdateRate = LTM_RATE_NORMAL
);

void telemetryInit(void)
{
#if defined(TELEMETRY_FRSKY)
    initFrSkyTelemetry();
#endif

#if defined(TELEMETRY_HOTT)
    initHoTTTelemetry();
#endif

#if defined(TELEMETRY_SMARTPORT)
    initSmartPortTelemetry();
#endif

#if defined(TELEMETRY_LTM)
    initLtmTelemetry();
#endif

#if defined(TELEMETRY_MAVLINK)
    initMAVLinkTelemetry();
#endif

#if defined(TELEMETRY_JETIEXBUS)
    initJetiExBusTelemetry();
#endif

#if defined(TELEMETRY_IBUS)
    initIbusTelemetry();
#endif

#if defined(TELEMETRY_CRSF)
    initCrsfTelemetry();
#endif

    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (telemetryConfig()->telemetry_switch)
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

#if defined(TELEMETRY_CRSF)
    checkCrsfTelemetryState();
#endif
}

void telemetryProcess(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs); // since not used by all the telemetry protocols

    #if defined(TELEMETRY_FRSKY)
    handleFrSkyTelemetry();
#endif

#if defined(TELEMETRY_HOTT)
    handleHoTTTelemetry(currentTimeUs);
#endif

#if defined(TELEMETRY_SMARTPORT)
    handleSmartPortTelemetry();
#endif

#if defined(TELEMETRY_LTM)
    handleLtmTelemetry();
#endif

#if defined(TELEMETRY_MAVLINK)
    handleMAVLinkTelemetry(currentTimeUs);
#endif

#if defined(TELEMETRY_JETIEXBUS)
    handleJetiExBusTelemetry();
#endif

#if defined(TELEMETRY_IBUS)
    handleIbusTelemetry();
#endif

#if defined(TELEMETRY_CRSF)
    handleCrsfTelemetry(currentTimeUs);
#endif
}

#endif
