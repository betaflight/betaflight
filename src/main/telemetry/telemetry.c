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

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "fc/config.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "msp/msp_serial.h"

#include "rx/rx.h"

#include "telemetry/telemetry.h"
#include "telemetry/frsky.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/mavlink.h"
#include "telemetry/crsf.h"
#include "telemetry/srxl.h"
#include "telemetry/ibus.h"
#include "telemetry/msp_shared.h"


PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inverted = false,
    .halfDuplex = 1,
    .telemetry_switch = 0,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = FRSKY_UNIT_METRICS,
    .frsky_vfas_precision = 0,
    .frsky_vfas_cell_voltage = 0,
    .hottAlarmSoundInterval = 5,
    .pidValuesAsTelemetry = 0,
    .report_cell_voltage = false
);

void telemetryInit(void)
{
#ifdef TELEMETRY_FRSKY
    initFrSkyTelemetry();
#endif
#ifdef TELEMETRY_HOTT
    initHoTTTelemetry();
#endif
#ifdef TELEMETRY_SMARTPORT
    initSmartPortTelemetry();
#endif
#ifdef TELEMETRY_LTM
    initLtmTelemetry();
#endif
#ifdef TELEMETRY_JETIEXBUS
    initJetiExBusTelemetry();
#endif
#ifdef TELEMETRY_MAVLINK
    initMAVLinkTelemetry();
#endif
#ifdef TELEMETRY_CRSF
    initCrsfTelemetry();
#endif
#ifdef TELEMETRY_SRXL
    initSrxlTelemetry();
#endif
#ifdef TELEMETRY_IBUS
    initIbusTelemetry();
#endif
    initSharedMsp();

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
    if (portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK) {
        return true;
    }
#ifdef TELEMETRY_IBUS
    if (   portConfig->functionMask & FUNCTION_TELEMETRY_IBUS
        && portConfig->functionMask & FUNCTION_RX_SERIAL
        && rxConfig()->serialrx_provider == SERIALRX_IBUS) {
        // IBUS serial RX & telemetry
        return true;
    }
#endif
    return false;
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
#ifdef TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef TELEMETRY_IBUS
    checkIbusTelemetryState();
#endif
}

void telemetryProcess(uint32_t currentTime)
{
#ifdef TELEMETRY_FRSKY
    handleFrSkyTelemetry();
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
#ifdef TELEMETRY_SRXL
    handleSrxlTelemetry(currentTime);
#endif
#ifdef TELEMETRY_IBUS
    handleIbusTelemetry();
#endif
}

#define TELEMETRY_FUNCTION_MASK (FUNCTION_TELEMETRY_FRSKY | FUNCTION_TELEMETRY_HOTT | FUNCTION_TELEMETRY_LTM | FUNCTION_TELEMETRY_SMARTPORT)

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspSerialReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_FUNCTION_MASK, FUNCTION_MSP);
    }
}
#endif
