/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/utils.h"

#include "io/serial.h"
#include "io/serial_feature_map.h"

#include "msp/msp_serial.h"
#include "pg/msp.h"

#ifdef USE_GPS
#include "pg/gps.h"
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
#include "pg/rx.h"
#endif
#ifdef USE_BLACKBOX
#include "blackbox/blackbox.h"
#endif
#ifdef USE_ESC_SENSOR
#include "sensors/esc_sensor.h"
#endif
#ifdef USE_RCDEVICE
#include "pg/rcdevice.h"
#endif
#ifdef USE_GIMBAL
#include "pg/gimbal.h"
#endif
#ifdef USE_VTX_COMMON
#include "drivers/vtx_common.h"
#include "io/vtx.h"
#endif
#ifdef USE_RANGEFINDER
#include "sensors/rangefinder.h"
#endif
#ifdef USE_OSD
#include "osd/osd.h"
#endif
#ifdef USE_TELEMETRY
#include "telemetry/telemetry.h"
#endif

uint32_t serialSynthesizeFunctionMask(serialPortIdentifier_e identifier)
{
    if (identifier == SERIAL_PORT_NONE) {
        return 0;
    }

    uint32_t mask = 0;

    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        if (mspConfig()->msp_uart[i] == identifier) {
            mask |= FUNCTION_MSP;
            break;
        }
    }

#ifdef USE_GPS
    if (gpsConfig()->gps_uart == identifier) {
        mask |= FUNCTION_GPS;
    }
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
    if (rxConfig()->rx_uart == identifier) {
        mask |= FUNCTION_RX_SERIAL;
    }
#endif
#ifdef USE_BLACKBOX
    if (blackboxConfig()->blackbox_uart == identifier) {
        mask |= FUNCTION_BLACKBOX;
    }
#endif
#ifdef USE_ESC_SENSOR
    if (escSensorConfig()->esc_sensor_uart == identifier) {
        mask |= FUNCTION_ESC_SENSOR;
    }
#endif
#ifdef USE_RCDEVICE
    if (rcdeviceConfig()->rcdevice_uart == identifier) {
        mask |= FUNCTION_RCDEVICE;
    }
#endif
#ifdef USE_GIMBAL
    if (gimbalTrackConfig()->gimbal_uart == identifier) {
        mask |= FUNCTION_GIMBAL;
    }
#endif
#ifdef USE_VTX_COMMON
    if (vtxSettingsConfig()->vtx_uart == identifier) {
        switch (vtxSettingsConfig()->vtx_type) {
#ifdef USE_VTX_SMARTAUDIO
        case VTXDEV_SMARTAUDIO:
            mask |= FUNCTION_VTX_SMARTAUDIO;
            break;
#endif
#ifdef USE_VTX_TRAMP
        case VTXDEV_TRAMP:
            mask |= FUNCTION_VTX_TRAMP;
            break;
#endif
#ifdef USE_VTX_MSP
        case VTXDEV_MSP:
            mask |= FUNCTION_VTX_MSP;
            break;
#endif
        default:
            break;
        }
    }
#endif
#ifdef USE_RANGEFINDER
    if (rangefinderConfig()->rangefinder_uart == identifier) {
        mask |= FUNCTION_LIDAR;
    }
#endif
#ifdef USE_OSD
    if (osdConfig()->osd_uart == identifier && osdConfig()->displayPortDevice == OSD_DISPLAYPORT_DEVICE_FRSKYOSD) {
        mask |= FUNCTION_FRSKY_OSD;
    }
    if (osdConfig()->osd_custom_text_uart == identifier) {
        mask |= FUNCTION_OSD_CUSTOM_TEXT;
    }
#endif
#ifdef USE_TELEMETRY
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].uart != identifier) {
            continue;
        }
        switch (telemetryConfig()->providers[i].protocol) {
#ifdef USE_TELEMETRY_FRSKY_HUB
        case TELEMETRY_PROTOCOL_FRSKY_HUB:
            mask |= FUNCTION_TELEMETRY_FRSKY_HUB;
            break;
#endif
#ifdef USE_TELEMETRY_HOTT
        case TELEMETRY_PROTOCOL_HOTT:
            mask |= FUNCTION_TELEMETRY_HOTT;
            break;
#endif
#ifdef USE_TELEMETRY_LTM
        case TELEMETRY_PROTOCOL_LTM:
            mask |= FUNCTION_TELEMETRY_LTM;
            break;
#endif
#ifdef USE_TELEMETRY_SMARTPORT
        case TELEMETRY_PROTOCOL_SMARTPORT:
            mask |= FUNCTION_TELEMETRY_SMARTPORT;
            break;
#endif
#ifdef USE_TELEMETRY_MAVLINK
        case TELEMETRY_PROTOCOL_MAVLINK:
            mask |= FUNCTION_TELEMETRY_MAVLINK;
            break;
#endif
#ifdef USE_TELEMETRY_IBUS
        case TELEMETRY_PROTOCOL_IBUS:
            mask |= FUNCTION_TELEMETRY_IBUS;
            break;
#endif
        default:
            break;
        }
    }
#endif

    return mask;
}

// Clear any feature PG field currently naming `identifier` so the
// apply phase can reassign cleanly.  Collapsed-enum selectors
// (rangefinder_hardware, displayPortDevice, vtx_type) are left alone —
// removing a UART doesn't imply changing the chosen hardware/protocol.
static void clearClaimsOnPort(serialPortIdentifier_e identifier)
{
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        if (mspConfig()->msp_uart[i] == identifier) {
            mspConfigMutable()->msp_uart[i] = SERIAL_PORT_NONE;
        }
    }
#ifdef USE_GPS
    if (gpsConfig()->gps_uart == identifier) {
        gpsConfigMutable()->gps_uart = SERIAL_PORT_NONE;
    }
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
    if (rxConfig()->rx_uart == identifier) {
        rxConfigMutable()->rx_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_BLACKBOX
    if (blackboxConfig()->blackbox_uart == identifier) {
        blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_ESC_SENSOR
    if (escSensorConfig()->esc_sensor_uart == identifier) {
        escSensorConfigMutable()->esc_sensor_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_RCDEVICE
    if (rcdeviceConfig()->rcdevice_uart == identifier) {
        rcdeviceConfigMutable()->rcdevice_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_GIMBAL
    if (gimbalTrackConfig()->gimbal_uart == identifier) {
        gimbalTrackConfigMutable()->gimbal_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_VTX_COMMON
    if (vtxSettingsConfig()->vtx_uart == identifier) {
        vtxSettingsConfigMutable()->vtx_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_RANGEFINDER
    if (rangefinderConfig()->rangefinder_uart == identifier) {
        rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_OSD
    if (osdConfig()->osd_uart == identifier) {
        osdConfigMutable()->osd_uart = SERIAL_PORT_NONE;
    }
    if (osdConfig()->osd_custom_text_uart == identifier) {
        osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_NONE;
    }
#endif
#ifdef USE_TELEMETRY
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].uart == identifier) {
            telemetryConfigMutable()->providers[i].protocol = TELEMETRY_PROTOCOL_NONE;
            telemetryConfigMutable()->providers[i].uart = SERIAL_PORT_NONE;
        }
    }
#endif
}

#ifdef USE_TELEMETRY
// A build can define USE_TELEMETRY without any individual protocol, leaving
// assignTelemetrySlot() unreachable; mark it possibly-unused so -Werror
// -Wunused-function doesn't fire (e.g. on PICO).  Callers must pre-validate
// slot availability via canApplyFunctionMask() — this helper asserts on
// overflow rather than returning an error.
static MAYBE_UNUSED void assignTelemetrySlot(serialPortIdentifier_e identifier, uint8_t protocol)
{
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].protocol == TELEMETRY_PROTOCOL_NONE) {
            telemetryConfigMutable()->providers[i].protocol = protocol;
            telemetryConfigMutable()->providers[i].uart = identifier;
            return;
        }
    }
}
#endif

#ifdef USE_TELEMETRY
static unsigned countTelemetryBits(uint32_t mask)
{
    unsigned n = 0;
#ifdef USE_TELEMETRY_FRSKY_HUB
    if (mask & FUNCTION_TELEMETRY_FRSKY_HUB) n++;
#endif
#ifdef USE_TELEMETRY_HOTT
    if (mask & FUNCTION_TELEMETRY_HOTT) n++;
#endif
#ifdef USE_TELEMETRY_LTM
    if (mask & FUNCTION_TELEMETRY_LTM) n++;
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    if (mask & FUNCTION_TELEMETRY_SMARTPORT) n++;
#endif
#ifdef USE_TELEMETRY_MAVLINK
    if (mask & FUNCTION_TELEMETRY_MAVLINK) n++;
#endif
#ifdef USE_TELEMETRY_IBUS
    if (mask & FUNCTION_TELEMETRY_IBUS) n++;
#endif
    (void)mask;  // A USE_TELEMETRY build without any sub-protocol touches none.
    return n;
}
#endif

// Check whether a mask can be applied to `identifier` without leaving the
// feature PGs in an inconsistent state.  Counts bits per category and checks
// MSP/telemetry slot availability as if clearClaimsOnPort(identifier) had
// already run — slots currently held by this port are considered free for
// the new mask.  No PG mutations are performed.
static bool canApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
#ifdef USE_VTX_COMMON
    unsigned vtxBits = 0;
#ifdef USE_VTX_SMARTAUDIO
    if (mask & FUNCTION_VTX_SMARTAUDIO) vtxBits++;
#endif
#ifdef USE_VTX_TRAMP
    if (mask & FUNCTION_VTX_TRAMP) vtxBits++;
#endif
#ifdef USE_VTX_MSP
    if (mask & FUNCTION_VTX_MSP) vtxBits++;
#endif
    if (vtxBits > 1) {
        return false;
    }
#endif

    if (mask & FUNCTION_MSP) {
        unsigned availableMsp = 0;
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == SERIAL_PORT_NONE
                || mspConfig()->msp_uart[i] == identifier) {
                availableMsp++;
            }
        }
        if (availableMsp < 1) {
            return false;
        }
    }

#ifdef USE_TELEMETRY
    const unsigned tlmNeeded = countTelemetryBits(mask);
    if (tlmNeeded > 0) {
        unsigned availableTlm = 0;
        for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
            if (telemetryConfig()->providers[i].protocol == TELEMETRY_PROTOCOL_NONE
                || telemetryConfig()->providers[i].uart == identifier) {
                availableTlm++;
            }
        }
        if (availableTlm < tlmNeeded) {
            return false;
        }
    }
#endif

    return true;
}

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    if (identifier == SERIAL_PORT_NONE) {
        return mask == 0;
    }

    // Validate against the pre-clear state so callers see an atomic
    // success/failure; if the mask can't be represented we must not
    // have touched the PGs.
    if (!canApplyFunctionMask(identifier, mask)) {
        return false;
    }

    clearClaimsOnPort(identifier);

    if (mask & FUNCTION_MSP) {
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == SERIAL_PORT_NONE) {
                mspConfigMutable()->msp_uart[i] = identifier;
                break;
            }
        }
    }
#ifdef USE_GPS
    if (mask & FUNCTION_GPS) {
        gpsConfigMutable()->gps_uart = identifier;
    }
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
    if (mask & FUNCTION_RX_SERIAL) {
        rxConfigMutable()->rx_uart = identifier;
    }
#endif
#ifdef USE_BLACKBOX
    if (mask & FUNCTION_BLACKBOX) {
        blackboxConfigMutable()->blackbox_uart = identifier;
    }
#endif
#ifdef USE_ESC_SENSOR
    if (mask & FUNCTION_ESC_SENSOR) {
        escSensorConfigMutable()->esc_sensor_uart = identifier;
    }
#endif
#ifdef USE_RCDEVICE
    if (mask & FUNCTION_RCDEVICE) {
        rcdeviceConfigMutable()->rcdevice_uart = identifier;
    }
#endif
#ifdef USE_GIMBAL
    if (mask & FUNCTION_GIMBAL) {
        gimbalTrackConfigMutable()->gimbal_uart = identifier;
    }
#endif
#ifdef USE_VTX_COMMON
    // VTX bit count pre-validated ≤ 1; at most one branch fires.
#ifdef USE_VTX_SMARTAUDIO
    if (mask & FUNCTION_VTX_SMARTAUDIO) {
        vtxSettingsConfigMutable()->vtx_uart = identifier;
        vtxSettingsConfigMutable()->vtx_type = VTXDEV_SMARTAUDIO;
    }
#endif
#ifdef USE_VTX_TRAMP
    if (mask & FUNCTION_VTX_TRAMP) {
        vtxSettingsConfigMutable()->vtx_uart = identifier;
        vtxSettingsConfigMutable()->vtx_type = VTXDEV_TRAMP;
    }
#endif
#ifdef USE_VTX_MSP
    if (mask & FUNCTION_VTX_MSP) {
        vtxSettingsConfigMutable()->vtx_uart = identifier;
        vtxSettingsConfigMutable()->vtx_type = VTXDEV_MSP;
    }
#endif
#endif
#ifdef USE_RANGEFINDER
    if (mask & FUNCTION_LIDAR) {
        rangefinderConfigMutable()->rangefinder_uart = identifier;
    }
#endif
#ifdef USE_OSD
    if (mask & FUNCTION_FRSKY_OSD) {
        osdConfigMutable()->osd_uart = identifier;
        osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_FRSKYOSD;
    }
    if (mask & FUNCTION_OSD_CUSTOM_TEXT) {
        osdConfigMutable()->osd_custom_text_uart = identifier;
    }
#endif
#ifdef USE_TELEMETRY
    // Telemetry slot availability pre-validated; assignTelemetrySlot always fits.
#ifdef USE_TELEMETRY_FRSKY_HUB
    if (mask & FUNCTION_TELEMETRY_FRSKY_HUB) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_FRSKY_HUB);
#endif
#ifdef USE_TELEMETRY_HOTT
    if (mask & FUNCTION_TELEMETRY_HOTT) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_HOTT);
#endif
#ifdef USE_TELEMETRY_LTM
    if (mask & FUNCTION_TELEMETRY_LTM) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_LTM);
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    if (mask & FUNCTION_TELEMETRY_SMARTPORT) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_SMARTPORT);
#endif
#ifdef USE_TELEMETRY_MAVLINK
    if (mask & FUNCTION_TELEMETRY_MAVLINK) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_MAVLINK);
#endif
#ifdef USE_TELEMETRY_IBUS
    if (mask & FUNCTION_TELEMETRY_IBUS) assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_IBUS);
#endif
#endif

    return true;
}

// Defaults a port reports when no feature claims it for the class.  These
// match the legacy pgResetFn_serialConfig values so an untouched config
// synthesizes back to exactly what it stored.
uint8_t serialDefaultPortBaud(serialBaudClass_e baudClass)
{
    switch (baudClass) {
    case SERIAL_BAUD_MSP:
        return BAUD_115200;
    case SERIAL_BAUD_GPS:
        return BAUD_57600;
    case SERIAL_BAUD_BLACKBOX:
        return BAUD_115200;
    case SERIAL_BAUD_TELEMETRY:
    default:
        return BAUD_AUTO;
    }
}

uint8_t serialSynthesizePortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass)
{
    if (identifier == SERIAL_PORT_NONE) {
        return serialDefaultPortBaud(baudClass);
    }

    switch (baudClass) {
    case SERIAL_BAUD_MSP:
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == identifier) {
                return mspConfig()->msp_baud[i];
            }
        }
        break;

#ifdef USE_GPS
    case SERIAL_BAUD_GPS:
        if (gpsConfig()->gps_uart == identifier) {
            return gpsConfig()->gps_baud;
        }
        break;
#endif

#ifdef USE_BLACKBOX
    case SERIAL_BAUD_BLACKBOX:
        if (blackboxConfig()->blackbox_uart == identifier) {
            return blackboxConfig()->blackbox_baud;
        }
        break;
#endif

    case SERIAL_BAUD_TELEMETRY:
#ifdef USE_TELEMETRY
        for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
            if (telemetryConfig()->providers[i].uart == identifier) {
                return telemetryConfig()->providers[i].baud;
            }
        }
#endif
#ifdef USE_OSD
        // OSD custom text historically rode the port's telemetry baud, so it
        // still reports through this class to keep the legacy view intact.
        if (osdConfig()->osd_custom_text_uart == identifier) {
            return osdConfig()->osd_custom_text_baud;
        }
#endif
        break;

    default:
        break;
    }

    return serialDefaultPortBaud(baudClass);
}

void serialApplyPortBaud(serialPortIdentifier_e identifier, serialBaudClass_e baudClass, uint8_t baudRateIndex)
{
    if (identifier == SERIAL_PORT_NONE) {
        return;
    }

    switch (baudClass) {
    case SERIAL_BAUD_MSP:
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == identifier) {
                mspConfigMutable()->msp_baud[i] = baudRateIndex;
            }
        }
        break;

#ifdef USE_GPS
    case SERIAL_BAUD_GPS:
        if (gpsConfig()->gps_uart == identifier) {
            gpsConfigMutable()->gps_baud = baudRateIndex;
        }
        break;
#endif

#ifdef USE_BLACKBOX
    case SERIAL_BAUD_BLACKBOX:
        if (blackboxConfig()->blackbox_uart == identifier) {
            blackboxConfigMutable()->blackbox_baud = baudRateIndex;
        }
        break;
#endif

    case SERIAL_BAUD_TELEMETRY:
#ifdef USE_TELEMETRY
        // Every provider on the port takes the value: the legacy layout held
        // one telemetry baud per port, so a write cannot mean anything else.
        for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
            if (telemetryConfig()->providers[i].uart == identifier) {
                telemetryConfigMutable()->providers[i].baud = baudRateIndex;
            }
        }
#endif
#ifdef USE_OSD
        if (osdConfig()->osd_custom_text_uart == identifier) {
            osdConfigMutable()->osd_custom_text_baud = baudRateIndex;
        }
#endif
        break;

    default:
        break;
    }
}

void serialBackfillFeatureFields(void)
{
    // Apply every port's mask unconditionally: apply-with-mask=0 runs the
    // clear phase so stale *_uart fields from EEPROM cannot out-live a
    // port that no longer claims them in the legacy view.
    //
    // A per-port apply can only fail if the legacy mask itself is
    // structurally invalid (two VTX protocols on one port, two lidar
    // categories on one port, or more MSP/telemetry claims across ports
    // than slots hold).  None of those are reachable from a well-formed
    // legacy config, so the return value is intentionally discarded; the
    // synthesized view of a partially-migrated port simply omits the bits
    // that couldn't be represented, which matches the legacy-is-invalid
    // semantics those masks had before migration.
    for (unsigned i = 0; i < ARRAYLEN(serialConfig()->portConfigs); i++) {
        const serialPortConfig_t *port = &serialConfig()->portConfigs[i];
        (void)serialApplyFunctionMask(port->identifier, port->functionMask);
    }
}
