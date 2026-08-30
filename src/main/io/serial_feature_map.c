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

#ifdef USE_VTX_COMMON
static uint32_t vtxFunctionMask(void)
{
    switch (vtxSettingsConfig()->vtx_type) {
#ifdef USE_VTX_SMARTAUDIO
    case VTXDEV_SMARTAUDIO:
        return FUNCTION_VTX_SMARTAUDIO;
#endif
#ifdef USE_VTX_TRAMP
    case VTXDEV_TRAMP:
        return FUNCTION_VTX_TRAMP;
#endif
#ifdef USE_VTX_MSP
    case VTXDEV_MSP:
        // An MSP VTX talks over MSP on its own UART, so it brings the MSP bit
        // with it; FUNCTION_VTX_MSP on its own is a conflict.
        return FUNCTION_VTX_MSP | FUNCTION_MSP;
#endif
    default:
        return 0;
    }
}
#endif

#ifdef USE_OSD
static uint32_t osdFunctionMask(void)
{
    switch (osdConfig()->displayPortDevice) {
    case OSD_DISPLAYPORT_DEVICE_FRSKYOSD:
        return FUNCTION_FRSKY_OSD;
#ifdef USE_MSP_DISPLAYPORT
    case OSD_DISPLAYPORT_DEVICE_MSP:
        return FUNCTION_MSP;
#endif
    default:
        return 0;
    }
}
#endif

#ifdef USE_TELEMETRY_PROVIDERS
static uint32_t telemetryProviderFunctionMask(unsigned providerIndex)
{
    switch (telemetryConfig()->providers[providerIndex].protocol) {
#ifdef USE_TELEMETRY_FRSKY_HUB
    case TELEMETRY_PROTOCOL_FRSKY_HUB:
        return FUNCTION_TELEMETRY_FRSKY_HUB;
#endif
#ifdef USE_TELEMETRY_HOTT
    case TELEMETRY_PROTOCOL_HOTT:
        return FUNCTION_TELEMETRY_HOTT;
#endif
#ifdef USE_TELEMETRY_LTM
    case TELEMETRY_PROTOCOL_LTM:
        return FUNCTION_TELEMETRY_LTM;
#endif
#ifdef USE_TELEMETRY_SMARTPORT
    case TELEMETRY_PROTOCOL_SMARTPORT:
        return FUNCTION_TELEMETRY_SMARTPORT;
#endif
#ifdef USE_TELEMETRY_MAVLINK
    case TELEMETRY_PROTOCOL_MAVLINK:
        return FUNCTION_TELEMETRY_MAVLINK;
#endif
#ifdef USE_TELEMETRY_IBUS
    case TELEMETRY_PROTOCOL_IBUS:
        return FUNCTION_TELEMETRY_IBUS;
#endif
    default:
        return 0;
    }
}
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
        mask |= vtxFunctionMask();
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
    if (osdConfig()->osd_uart == identifier) {
        mask |= osdFunctionMask();
    }
    if (osdConfig()->osd_custom_text_uart == identifier) {
        mask |= FUNCTION_OSD_CUSTOM_TEXT;
    }
#endif
#ifdef USE_TELEMETRY_PROVIDERS
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        if (telemetryConfig()->providers[i].uart == identifier) {
            mask |= telemetryProviderFunctionMask(i);
        }
    }
#endif

    return mask;
}

unsigned serialGetPortClaims(serialPortIdentifier_e identifier, serialPortClaim_t *claims, unsigned maxClaims)
{
    unsigned count = 0;

    if (identifier == SERIAL_PORT_NONE) {
        return 0;
    }

#define ADD_CLAIM(claimName, mask) \
    do { \
        if (count < maxClaims) { \
            claims[count].name = (claimName); \
            claims[count].functionMask = (mask); \
            count++; \
        } \
    } while (0)

    static const char * const mspClaimNames[] = { "msp_1", "msp_2", "msp_3" };
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT && i < ARRAYLEN(mspClaimNames); i++) {
        if (mspConfig()->msp_uart[i] == identifier) {
            ADD_CLAIM(mspClaimNames[i], FUNCTION_MSP);
        }
    }

#ifdef USE_GPS
    if (gpsConfig()->gps_uart == identifier) {
        ADD_CLAIM("gps", FUNCTION_GPS);
    }
#endif
#if defined(USE_RX_PWM) || defined(USE_RX_PPM) || defined(USE_SERIALRX) || defined(USE_RX_MSP) || defined(USE_RX_SPI)
    if (rxConfig()->rx_uart == identifier) {
        ADD_CLAIM("rx", FUNCTION_RX_SERIAL);
    }
#endif
#ifdef USE_BLACKBOX
    if (blackboxConfig()->blackbox_uart == identifier) {
        ADD_CLAIM("blackbox", FUNCTION_BLACKBOX);
    }
#endif
#ifdef USE_ESC_SENSOR
    if (escSensorConfig()->esc_sensor_uart == identifier) {
        ADD_CLAIM("esc_sensor", FUNCTION_ESC_SENSOR);
    }
#endif
#ifdef USE_RCDEVICE
    if (rcdeviceConfig()->rcdevice_uart == identifier) {
        ADD_CLAIM("rcdevice", FUNCTION_RCDEVICE);
    }
#endif
#ifdef USE_GIMBAL
    if (gimbalTrackConfig()->gimbal_uart == identifier) {
        ADD_CLAIM("gimbal", FUNCTION_GIMBAL);
    }
#endif
#ifdef USE_VTX_COMMON
    if (vtxSettingsConfig()->vtx_uart == identifier) {
        ADD_CLAIM("vtx", vtxFunctionMask());
    }
#endif
#ifdef USE_RANGEFINDER
    if (rangefinderConfig()->rangefinder_uart == identifier) {
        uint32_t mask = FUNCTION_LIDAR;
#ifdef USE_RANGEFINDER_MT
        // An MSP-transport module's port opens as an implied MSP port, so
        // that is the function the claim is active under.
        if (rangefinderTypeUsesMsp(rangefinderConfig()->rangefinder_hardware)) {
            mask |= FUNCTION_MSP;
        }
#endif
        ADD_CLAIM("rangefinder", mask);
    }
#endif
#ifdef USE_OPTICALFLOW
    if (opticalflowConfig()->opticalflow_uart == identifier) {
        uint32_t mask = FUNCTION_LIDAR;
#ifdef USE_OPTICALFLOW_MT
        if (opticalflowTypeUsesMsp(opticalflowConfig()->opticalflow_hardware)) {
            mask |= FUNCTION_MSP;
        }
#endif
        ADD_CLAIM("opticalflow", mask);
    }
#endif
#ifdef USE_OSD
    if (osdConfig()->osd_uart == identifier) {
        ADD_CLAIM("osd", osdFunctionMask());
    }
    if (osdConfig()->osd_custom_text_uart == identifier) {
        ADD_CLAIM("osd_custom_text", FUNCTION_OSD_CUSTOM_TEXT);
    }
#endif
#ifdef USE_TELEMETRY_PROVIDERS
    static const char * const telemetryClaimNames[] = { "telemetry_1", "telemetry_2", "telemetry_3" };
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS && i < ARRAYLEN(telemetryClaimNames); i++) {
        if (telemetryConfig()->providers[i].uart == identifier) {
            ADD_CLAIM(telemetryClaimNames[i], telemetryProviderFunctionMask(i));
        }
    }
#endif

#undef ADD_CLAIM

    return count;
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

#if IMPLIED_MSP_PORT_COUNT > 0
static void addImpliedMspPort(serialPortIdentifier_e *ports, unsigned *count, unsigned maxPorts,
                              serialPortIdentifier_e identifier)
{
    if (identifier == SERIAL_PORT_NONE || *count >= maxPorts) {
        return;
    }

    for (unsigned i = 0; i < *count; i++) {
        if (ports[i] == identifier) {
            return;
        }
    }

    ports[(*count)++] = identifier;
}
#endif

unsigned serialImpliedMspPorts(serialPortIdentifier_e *ports, unsigned maxPorts)
{
    unsigned count = 0;

#ifdef USE_RANGEFINDER_MT
    if (rangefinderTypeUsesMsp(rangefinderConfig()->rangefinder_hardware)) {
        addImpliedMspPort(ports, &count, maxPorts, rangefinderConfig()->rangefinder_uart);
    }
#endif
#ifdef USE_OPTICALFLOW_MT
    if (opticalflowTypeUsesMsp(opticalflowConfig()->opticalflow_hardware)) {
        addImpliedMspPort(ports, &count, maxPorts, opticalflowConfig()->opticalflow_uart);
    }
#endif
#if defined(USE_OSD) && defined(USE_MSP_DISPLAYPORT)
    if (osdConfig()->displayPortDevice == OSD_DISPLAYPORT_DEVICE_MSP) {
        addImpliedMspPort(ports, &count, maxPorts, osdConfig()->osd_uart);
    }
#endif
#ifdef USE_VTX_MSP
    if (vtxSettingsConfig()->vtx_type == VTXDEV_MSP) {
        addImpliedMspPort(ports, &count, maxPorts, vtxSettingsConfig()->vtx_uart);
    }
#endif
#if IMPLIED_MSP_PORT_COUNT == 0
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
    // Its baud comes back to the class default too: the rate that came with the
    // rejected configuration is no use on a board whose first port is a UART.
    mspConfigMutable()->msp_uart[0] = serialPortIdentifiers[0];
    mspConfigMutable()->msp_baud[0] = serialDefaultPortBaud(SERIAL_BAUD_MSP);
}

