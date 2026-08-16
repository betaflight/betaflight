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
#ifdef USE_OPTICALFLOW
#include "sensors/opticalflow.h"
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
#ifdef USE_OPTICALFLOW
    // One module can answer as both sensors, so the bit is shared rather than
    // held per feature; ORing it twice for one port is the same as once.
    if (opticalflowConfig()->opticalflow_uart == identifier) {
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
#ifdef USE_TELEMETRY_PROVIDERS
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
// `keepMsp` spares an MSP claim on the port, so resolving a conflict cannot
// take away the link the board is being configured over.
static void clearClaimsOnPort(serialPortIdentifier_e identifier, bool keepMsp)
{
    if (!keepMsp) {
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == identifier) {
                mspConfigMutable()->msp_uart[i] = SERIAL_PORT_NONE;
            }
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
#ifdef USE_OPTICALFLOW
    if (opticalflowConfig()->opticalflow_uart == identifier) {
        opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_NONE;
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
#ifdef USE_TELEMETRY_PROVIDERS
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].uart == identifier) {
            telemetryConfigMutable()->providers[i].protocol = TELEMETRY_PROTOCOL_NONE;
            telemetryConfigMutable()->providers[i].uart = SERIAL_PORT_NONE;
        }
    }
#endif
}

unsigned serialImpliedMspPorts(serialPortIdentifier_e *ports, unsigned maxPorts)
{
    unsigned count = 0;

#ifdef USE_RANGEFINDER_MT
    if (rangefinderTypeUsesMsp(rangefinderConfig()->rangefinder_hardware)
        && rangefinderConfig()->rangefinder_uart != SERIAL_PORT_NONE
        && count < maxPorts) {
        ports[count++] = rangefinderConfig()->rangefinder_uart;
    }
#endif
#ifdef USE_OPTICALFLOW_MT
    if (opticalflowTypeUsesMsp(opticalflowConfig()->opticalflow_hardware)
        && opticalflowConfig()->opticalflow_uart != SERIAL_PORT_NONE
        && count < maxPorts) {
        const serialPortIdentifier_e identifier = opticalflowConfig()->opticalflow_uart;

        bool alreadyClaimed = false;
        for (unsigned i = 0; i < count; i++) {
            if (ports[i] == identifier) {
                alreadyClaimed = true;
                break;
            }
        }

        if (!alreadyClaimed) {
            ports[count++] = identifier;
        }
    }
#endif
#if !defined(USE_RANGEFINDER_MT) && !defined(USE_OPTICALFLOW_MT)
    UNUSED(ports);
    UNUSED(maxPorts);
#endif

    return count;
}

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
#ifdef USE_TELEMETRY_PROVIDERS
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

void serialDropConflictingAssignments(void)
{
    for (unsigned i = 0; i < ARRAYLEN(serialPortIdentifiers); i++) {
        const serialPortIdentifier_e identifier = serialPortIdentifiers[i];

        if (serialPortFunctionsConflict(identifier)) {
            clearClaimsOnPort(identifier, true);
        }
    }
}

void serialResetFeatureAssignments(void)
{
    for (unsigned i = 0; i < ARRAYLEN(serialPortIdentifiers); i++) {
        clearClaimsOnPort(serialPortIdentifiers[i], false);
    }

    // The first port stays MSP so the board remains reachable after a reset.
    mspConfigMutable()->msp_uart[0] = serialPortIdentifiers[0];
}

