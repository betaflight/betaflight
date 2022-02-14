/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "common/utils.h"
#include "common/unit.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"

#include "io/serial.h"

#include "config/config.h"
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
#include "telemetry/ghst.h"
#include "telemetry/srxl.h"
#include "telemetry/ibus.h"
#include "telemetry/msp_shared.h"

PG_REGISTER_WITH_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 5);

PG_RESET_TEMPLATE(telemetryConfig_t, telemetryConfig,
    .telemetry_inverted = false,
    .halfDuplex = 1,
    .gpsNoFixLatitude = 0,
    .gpsNoFixLongitude = 0,
    .frsky_coordinate_format = FRSKY_FORMAT_DMS,
    .frsky_unit = UNIT_METRIC,
    .frsky_vfas_precision = 0,
    .hottAlarmSoundInterval = 5,
    .pidValuesAsTelemetry = 0,
    .report_cell_voltage = false,
    .flysky_sensors = {
            IBUS_SENSOR_TYPE_TEMPERATURE,
            IBUS_SENSOR_TYPE_RPM_FLYSKY,
            IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE
    },
    .disabledSensors = ESC_SENSOR_ALL | SENSOR_CAP_USED,
    .mavlink_mah_as_heading_divisor = 0,
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
#ifdef USE_TELEMETRY_GHST
    initGhstTelemetry();
#endif
#ifdef USE_TELEMETRY_CRSF
    initCrsfTelemetry();
#if defined(USE_MSP_OVER_TELEMETRY)
    initCrsfMspBuffer();
#endif
#endif
#ifdef USE_TELEMETRY_SRXL
    initSrxlTelemetry();
#endif
#ifdef USE_TELEMETRY_IBUS
    initIbusTelemetry();
#endif
#if defined(USE_MSP_OVER_TELEMETRY)
    initSharedMsp();
#endif

    telemetryCheckState();
}

bool telemetryDetermineEnabledState(portSharing_e portSharing)
{
    bool enabled = portSharing == PORTSHARING_NOT_SHARED;

    if (portSharing == PORTSHARING_SHARED) {
        if (isModeActivationConditionPresent(BOXTELEMETRY))
            enabled = IS_RC_MODE_ACTIVE(BOXTELEMETRY);
        else
            enabled = ARMING_FLAG(ARMED);
    }

    return enabled;
}

bool telemetryCheckRxPortShared(const serialPortConfig_t *portConfig, const SerialRXType serialrxProvider)
{
    if (portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & TELEMETRY_SHAREABLE_PORT_FUNCTIONS_MASK &&
        (serialrxProvider == SERIALRX_SPEKTRUM1024 ||
        serialrxProvider == SERIALRX_SPEKTRUM2048 ||
        serialrxProvider == SERIALRX_SBUS ||
        serialrxProvider == SERIALRX_SUMD ||
        serialrxProvider == SERIALRX_SUMH ||
        serialrxProvider == SERIALRX_XBUS_MODE_B ||
        serialrxProvider == SERIALRX_XBUS_MODE_B_RJ01 ||
        serialrxProvider == SERIALRX_IBUS)) {

        return true;
    }
#ifdef USE_TELEMETRY_IBUS
    if (portConfig->functionMask & FUNCTION_TELEMETRY_IBUS
        && portConfig->functionMask & FUNCTION_RX_SERIAL
        && serialrxProvider == SERIALRX_IBUS) {
        // IBUS serial RX & telemetry
        return true;
    }
#endif
#if defined(USE_MSP_OVER_TELEMETRY) && defined(USE_VTX_MSP)
    if (portConfig->functionMask & FUNCTION_RX_SERIAL && portConfig->functionMask & FUNCTION_VTX_MSP) {
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
#ifdef USE_TELEMETRY_GHST
    checkGhstTelemetryState();
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
#ifdef USE_TELEMETRY_GHST
    handleGhstTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_SRXL
    handleSrxlTelemetry(currentTime);
#endif
#ifdef USE_TELEMETRY_IBUS
    handleIbusTelemetry();
#endif
}

bool telemetryIsSensorEnabled(sensor_e sensor)
{
    return ~(telemetryConfig()->disabledSensors) & sensor;
}
#endif
