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

#include <platform.h>

#ifdef TELEMETRY

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/gpio.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "fc/runtime_config.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/fc_serial.h"
#include "io/serial.h"

#include "rx/rx.h"


#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"

PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);

#ifdef STM32F303xC
// hardware supports serial port inversion, make users life easier for those that want to connect SBus RX's
#define DEFAULT_TELEMETRY_INVERSION 1
#else
#define DEFAULT_TELEMETRY_INVERSION 0
#endif


PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inversion = DEFAULT_TELEMETRY_INVERSION,
);

void telemetryInit(void)
{
    initFrSkyTelemetry();
    initHoTTTelemetry();
    initSmartPortTelemetry();
    initLtmTelemetry();
    initMAVLinkTelemetry();
    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (telemetryConfig()->telemetry_switch)
            enabled = rcModeIsActive(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

void telemetryCheckState(void)
{
    checkFrSkyTelemetryState();
    checkHoTTTelemetryState();
    checkSmartPortTelemetryState();
    checkLtmTelemetryState();
    checkMAVLinkTelemetryState();
}

void telemetryProcess(uint16_t deadband3d_throttle)
{
    handleFrSkyTelemetry(deadband3d_throttle);
    handleHoTTTelemetry();
    handleSmartPortTelemetry();
    handleLtmTelemetry();
    handleMAVLinkTelemetry();
}

#endif
