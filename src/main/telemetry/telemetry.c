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

#ifdef USE_TELEMETRY

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

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
#include "telemetry/frsky_hub.h"
#include "telemetry/hott.h"
#include "telemetry/smartport.h"
#include "telemetry/ltm.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/mavlink.h"
#include "telemetry/crsf.h"
#include "telemetry/srxl.h"
#include "telemetry/ibus.h"
#include "telemetry/msp_shared.h"

PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 1);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inverted = false,
    .halfDuplex = 1,
    .telemetry_switch = 0,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = FRSKY_UNIT_METRICS,
    .frsky_vfas_precision = 0,
    .hottAlarmSoundInterval = 5,
    .pidValuesAsTelemetry = 0,
    .report_cell_voltage = false
);

void telemetryInit(void)
{
#ifdef USE_TELEMETRY_FRSKY_HUB
    initFrSkyHubTelemetry();
#endif
#ifdef USE_TELEMETRY_HOTT
    initHoTTTelemetry();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    initSmartPortTelemetry();
#endif
#ifdef USE_TELEMETRY_LTM
    initLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    initJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    initMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    initCrsfTelemetry();
#endif
#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif
#ifdef USE_TELEMETRY_IBUS
    initIbusTelemetry();
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    initSharedMsp();
    initCrsfMspBuffer();
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
    if (portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK &&
        (rxConfig()->serialrx_provider == SERIALRX_SPEKTRUM1024 ||
        rxConfig()->serialrx_provider == SERIALRX_SPEKTRUM2048 ||
        rxConfig()->serialrx_provider == SERIALRX_SBUS ||
        rxConfig()->serialrx_provider == SERIALRX_SUMD ||
        rxConfig()->serialrx_provider == SERIALRX_SUMH ||
        rxConfig()->serialrx_provider == SERIALRX_XBUS_MODE_B ||
        rxConfig()->serialrx_provider == SERIALRX_XBUS_MODE_B_RJ01 ||
        rxConfig()->serialrx_provider == SERIALRX_IBUS)) {

        return true;
    }
#ifdef USE_TELEMETRY_IBUS
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
#ifdef USE_TELEMETRY_FRSKY_HUB
    checkFrSkyHubTelemetryState();
#endif
#ifdef USE_TELEMETRY_HOTT
    checkHoTTTelemetryState();
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    checkSmartPortTelemetryState();
#endif
#ifdef USE_TELEMETRY_LTM
    checkLtmTelemetryState();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    checkJetiExBusTelemetryState();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    checkMAVLinkTelemetryState();
#endif
#ifdef USE_TELEMETRY_CRSF
    checkCrsfTelemetryState();
#endif
#ifdef USE_TELEMETRY_SRXL
    checkSrxlTelemetryState();
#endif
#ifdef USE_TELEMETRY_IBUS
    checkIbusTelemetryState();
#endif
}

void telemetryProcess(uint32_t currentTime)
{
#ifdef USE_TELEMETRY_FRSKY_HUB
    handleFrSkyHubTelemetry(currentTime);
#else
    UNUSED(currentTime);
#endif
#ifdef USE_TELEMETRY_HOTT
    handleHoTTTelemetry(currentTime);
#else
    UNUSED(currentTime);
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    handleSmartPortTelemetry();
#endif
#ifdef USE_TELEMETRY_LTM
    handleLtmTelemetry();
#endif
#ifdef USE_TELEMETRY_JETIEXBUS
    handleJetiExBusTelemetry();
#endif
#ifdef USE_TELEMETRY_MAVLINK
    handleMAVLinkTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    handleCrsfTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_IBUS
    handleIbusTelemetry();
#endif
}

void releaseSharedTelemetryPorts(void) {
    serialPort_t *sharedPort = findSharedSerialPort(TELEMETRY_PORT_FUNCTIONS_MASK, FUNCTION_MSP);
    while (sharedPort) {
        mspSerialReleasePortIfAllocated(sharedPort);
        sharedPort = findNextSharedSerialPort(TELEMETRY_PORT_FUNCTIONS_MASK, FUNCTION_MSP);
    }
}
#endif
