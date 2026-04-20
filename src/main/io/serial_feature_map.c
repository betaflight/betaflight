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
        switch (rangefinderConfig()->rangefinder_hardware) {
        case RANGEFINDER_TFMINI:
        case RANGEFINDER_TF02:
        case RANGEFINDER_TFNOVA:
        case RANGEFINDER_UPT1:
            mask |= FUNCTION_LIDAR_TF;
            break;
        case RANGEFINDER_NOOPLOOP_F2:
        case RANGEFINDER_NOOPLOOP_F2P:
        case RANGEFINDER_NOOPLOOP_F2PH:
        case RANGEFINDER_NOOPLOOP_F:
        case RANGEFINDER_NOOPLOOP_FP:
        case RANGEFINDER_NOOPLOOP_F2MINI:
            mask |= FUNCTION_LIDAR_NL;
            break;
        default:
            break;
        }
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
static bool assignTelemetrySlot(serialPortIdentifier_e identifier, uint8_t protocol)
{
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].protocol == TELEMETRY_PROTOCOL_NONE) {
            telemetryConfigMutable()->providers[i].protocol = protocol;
            telemetryConfigMutable()->providers[i].uart = identifier;
            return true;
        }
    }
    return false;
}
#endif

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    if (identifier == SERIAL_PORT_NONE) {
        return mask == 0;
    }

    clearClaimsOnPort(identifier);

    bool ok = true;

    if (mask & FUNCTION_MSP) {
        bool placed = false;
        for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
            if (mspConfig()->msp_uart[i] == SERIAL_PORT_NONE) {
                mspConfigMutable()->msp_uart[i] = identifier;
                placed = true;
                break;
            }
        }
        if (!placed) {
            ok = false;
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
    {
        // At most one VTX bit may be set; later bits overwrite earlier ones and flag a conflict.
        uint8_t vtxBits = 0;
#ifdef USE_VTX_SMARTAUDIO
        if (mask & FUNCTION_VTX_SMARTAUDIO) {
            vtxSettingsConfigMutable()->vtx_uart = identifier;
            vtxSettingsConfigMutable()->vtx_type = VTXDEV_SMARTAUDIO;
            vtxBits++;
        }
#endif
#ifdef USE_VTX_TRAMP
        if (mask & FUNCTION_VTX_TRAMP) {
            vtxSettingsConfigMutable()->vtx_uart = identifier;
            vtxSettingsConfigMutable()->vtx_type = VTXDEV_TRAMP;
            vtxBits++;
        }
#endif
#ifdef USE_VTX_MSP
        if (mask & FUNCTION_VTX_MSP) {
            vtxSettingsConfigMutable()->vtx_uart = identifier;
            vtxSettingsConfigMutable()->vtx_type = VTXDEV_MSP;
            vtxBits++;
        }
#endif
        if (vtxBits > 1) {
            ok = false;
        }
    }
#endif
#ifdef USE_RANGEFINDER
    {
        uint8_t lidarBits = 0;
        if (mask & FUNCTION_LIDAR_TF) {
            rangefinderConfigMutable()->rangefinder_uart = identifier;
            switch (rangefinderConfig()->rangefinder_hardware) {
            case RANGEFINDER_TFMINI:
            case RANGEFINDER_TF02:
            case RANGEFINDER_TFNOVA:
            case RANGEFINDER_UPT1:
                break;  // compatible; leave alone
            default:
                rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
                break;
            }
            lidarBits++;
        }
        if (mask & FUNCTION_LIDAR_NL) {
            rangefinderConfigMutable()->rangefinder_uart = identifier;
            switch (rangefinderConfig()->rangefinder_hardware) {
            case RANGEFINDER_NOOPLOOP_F2:
            case RANGEFINDER_NOOPLOOP_F2P:
            case RANGEFINDER_NOOPLOOP_F2PH:
            case RANGEFINDER_NOOPLOOP_F:
            case RANGEFINDER_NOOPLOOP_FP:
            case RANGEFINDER_NOOPLOOP_F2MINI:
                break;
            default:
                rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NOOPLOOP_F2;
                break;
            }
            lidarBits++;
        }
        if (lidarBits > 1) {
            ok = false;
        }
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
#ifdef USE_TELEMETRY_FRSKY_HUB
    if (mask & FUNCTION_TELEMETRY_FRSKY_HUB) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_FRSKY_HUB);
    }
#endif
#ifdef USE_TELEMETRY_HOTT
    if (mask & FUNCTION_TELEMETRY_HOTT) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_HOTT);
    }
#endif
#ifdef USE_TELEMETRY_LTM
    if (mask & FUNCTION_TELEMETRY_LTM) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_LTM);
    }
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    if (mask & FUNCTION_TELEMETRY_SMARTPORT) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_SMARTPORT);
    }
#endif
#ifdef USE_TELEMETRY_MAVLINK
    if (mask & FUNCTION_TELEMETRY_MAVLINK) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_MAVLINK);
    }
#endif
#ifdef USE_TELEMETRY_IBUS
    if (mask & FUNCTION_TELEMETRY_IBUS) {
        ok &= assignTelemetrySlot(identifier, TELEMETRY_PROTOCOL_IBUS);
    }
#endif
#endif

    return ok;
}
