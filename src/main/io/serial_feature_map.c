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

/*
 * FUNCTION_LIDAR names the UART the rangefinder / optical flow module is wired
 * to.  It does not say how the module talks, and the two transports differ in
 * who opens the port:
 *
 *   TF / Nooploop / UPT1  - the driver calls openSerialPort() itself during
 *                           sensorsAutodetect().
 *   MT family             - rangefinder_lidarmt.c is a bridge over MSP.  The
 *                           module pushes MSP2_SENSOR_RANGEFINDER_LIDARMT /
 *                           MSP2_SENSOR_OPTICALFLOW_MT frames, which are
 *                           handled by the ordinary MSP command dispatcher, so
 *                           the declared port is only heard once an MSP port
 *                           has been opened on it.
 *
 * True for the second case, i.e. when the sensor port needs an MSP port.
 *
 * A pair of selections that disagree - an MT rangefinder alongside a UPT1
 * optical flow, say - is not a real wiring: there is one declared port and one
 * module on it.  The native driver wins, because it claims the port during
 * sensorsAutodetect() and openSerialPort() then refuses the MSP open.  Say so
 * here rather than leaning on that ordering, so the capacity check below does
 * not reserve an MSP slot the port will never use.
 */

#ifdef USE_RANGEFINDER
// The rangefinder drivers that open the sensor UART themselves.  Enumerated
// rather than derived as "anything that is not MT", because the two mistakes
// are not equally bad: calling a serial driver MSP-transport merely attempts
// an open that openSerialPort() refuses, while calling an MSP one serial
// suppresses the port it needs and the sensor goes quiet.  A complement would
// silently sort a future I2C or CAN rangefinder into the second case.
static bool rangefinderOpensPortItself(void)
{
    switch (rangefinderConfig()->rangefinder_hardware) {
    case RANGEFINDER_TFMINI:
    case RANGEFINDER_TF02:
    case RANGEFINDER_TFNOVA:
    case RANGEFINDER_NOOPLOOP_F2:
    case RANGEFINDER_NOOPLOOP_F2P:
    case RANGEFINDER_NOOPLOOP_F2PH:
    case RANGEFINDER_NOOPLOOP_F:
    case RANGEFINDER_NOOPLOOP_FP:
    case RANGEFINDER_NOOPLOOP_F2MINI:
    case RANGEFINDER_UPT1:
        return true;
    default:
        // RANGEFINDER_NONE claims no UART, and HCSR04 is pin-driven.
        return false;
    }
}
#endif

bool serialSensorPortUsesMsp(void)
{
    bool opensPortItself = false;
    bool reportsOverMsp = false;

#ifdef USE_RANGEFINDER
    opensPortItself |= rangefinderOpensPortItself();
#ifdef USE_RANGEFINDER_MT
    switch (rangefinderConfig()->rangefinder_hardware) {
    case RANGEFINDER_MTF01:
    case RANGEFINDER_MTF02:
    case RANGEFINDER_MTF01P:
    case RANGEFINDER_MTF02P:
        reportsOverMsp = true;
        break;
    default:
        break;
    }
#endif
#endif

#ifdef USE_OPTICALFLOW
    opensPortItself |= (opticalflowConfig()->opticalflow_hardware == OPTICALFLOW_UPT1);
#ifdef USE_OPTICALFLOW_MT
    reportsOverMsp |= (opticalflowConfig()->opticalflow_hardware == OPTICALFLOW_MT);
#endif
#endif

    return reportsOverMsp && !opensPortItself;
}

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

#if defined(USE_RANGEFINDER)
// Whether `sensorPort` is already accounted for as an explicit MSP port in the
// post-apply configuration, so the implied allocation must not be counted a
// second time.  Slots held by `identifier` are about to be cleared, so only
// this mask speaks for that port.
static bool sensorPortHoldsMspSlot(serialPortIdentifier_e sensorPort,
                                   serialPortIdentifier_e identifier, uint32_t mask)
{
    if (sensorPort == identifier) {
        return (mask & FUNCTION_MSP) != 0;
    }

    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        if (mspConfig()->msp_uart[i] == sensorPort) {
            return true;
        }
    }

    return false;
}
#endif

// What the MSP count below would come to if this apply changed nothing, i.e.
// the pressure the configuration is already under.
static unsigned currentMspPressure(void)
{
    unsigned used = 0;
#if defined(USE_RANGEFINDER)
    const serialPortIdentifier_e sensorPort =
        (serialPortIdentifier_e)rangefinderConfig()->rangefinder_uart;
    bool sensorHoldsSlot = false;
#endif

    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        if (mspConfig()->msp_uart[i] == SERIAL_PORT_NONE) {
            continue;
        }
        used++;
#if defined(USE_RANGEFINDER)
        if (mspConfig()->msp_uart[i] == sensorPort) {
            sensorHoldsSlot = true;
        }
#endif
    }

#if defined(USE_RANGEFINDER)
    if (serialSensorPortUsesMsp() && sensorPort != SERIAL_PORT_NONE && !sensorHoldsSlot) {
        used++;
    }
#endif

    return used;
}

// Check whether a mask can be applied to `identifier` without leaving the
// feature PGs in an inconsistent state.  Counts bits per category and checks
// MSP/telemetry slot availability as if clearClaimsOnPort(identifier) had
// already run — slots currently held by this port are considered free for
// the new mask.  No PG mutations are performed.
//
// `reserveImpliedSensorPort` keeps a slot for an MSP-transport sensor that
// claims none of its own; see the MSP capacity block.  The EEPROM migration
// passes false: a config that is already over budget boots today with a deaf
// sensor, and rejecting one of its ports there would drop that port's feature
// claims instead, which is strictly worse.
static bool canApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask,
                                 bool reserveImpliedSensorPort)
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

    // MSP capacity, counted over the whole configuration this apply would
    // leave behind rather than over this mask alone.  mspPorts[] is sized to
    // MAX_MSP_PORT_COUNT like msp_uart[] is, and an MSP-transport sensor is
    // given one of those entries at boot from FUNCTION_LIDAR while claiming no
    // msp_uart[] slot of its own.  Counting only the mask meant the sensor
    // could be assigned first and every remaining slot handed out afterwards,
    // each write passing, and the sensor left silent at boot.
    //
    // This guards the write paths; it is not an invariant.  A later change of
    // rangefinder_hardware from a native serial driver to an MT one does not
    // come through here and can still strand the sensor - reassigning its port
    // recovers it.
    unsigned mspUsed = 0;
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        if (mspConfig()->msp_uart[i] != SERIAL_PORT_NONE
            && mspConfig()->msp_uart[i] != identifier) {
            mspUsed++;
        }
    }
    if (mask & FUNCTION_MSP) {
        mspUsed++;
    }

#if defined(USE_RANGEFINDER)
    if (reserveImpliedSensorPort && serialSensorPortUsesMsp()) {
        // clearClaimsOnPort() releases rangefinder_uart only when it names
        // `identifier`, so derive the port this apply leaves behind rather
        // than reading the current one - otherwise removing FUNCTION_LIDAR
        // from the sensor's own port still reserves a slot for it.
        const serialPortIdentifier_e sensorPort = (mask & FUNCTION_LIDAR)
            ? identifier
            : (rangefinderConfig()->rangefinder_uart == identifier
                ? SERIAL_PORT_NONE
                : (serialPortIdentifier_e)rangefinderConfig()->rangefinder_uart);

        if (sensorPort != SERIAL_PORT_NONE
            && !sensorPortHoldsMspSlot(sensorPort, identifier, mask)) {
            mspUsed++;
        }
    }
#else
    (void)reserveImpliedSensorPort;
#endif

    // Refuse what makes the pressure worse, not merely what is over budget.
    // A configuration can go over budget behind this check's back, because
    // rangefinder_hardware is not written through here: assign the sensor port
    // while the sensor is still NONE, then pick an MT module on the Sensors
    // tab.  Refusing every write from there would freeze the Ports tab - its
    // first record is the USB VCP, which isSerialConfigValid() requires to
    // carry MSP, so the later record that would free a slot is never reached
    // and firmware rejects the whole save.  Allowing writes that hold or
    // lower the count leaves the user a way out.
    if (mspUsed > MAX_MSP_PORT_COUNT && mspUsed > currentMspPressure()) {
        return false;
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

static bool applyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask,
                              bool reserveImpliedSensorPort)
{
    if (identifier == SERIAL_PORT_NONE) {
        return mask == 0;
    }

    // Validate against the pre-clear state so callers see an atomic
    // success/failure; if the mask can't be represented we must not
    // have touched the PGs.
    if (!canApplyFunctionMask(identifier, mask, reserveImpliedSensorPort)) {
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

bool serialApplyFunctionMask(serialPortIdentifier_e identifier, uint32_t mask)
{
    return applyFunctionMask(identifier, mask, true);
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
    //
    // The implied MSP port for an MSP-transport sensor is deliberately not
    // reserved here.  A stored config that is already over budget - every MSP
    // slot taken and an MT sensor on a further port - is well-formed and boots
    // today with a deaf sensor.  Reserving during migration would reject
    // whichever port happens to be applied last, dropping that port's feature
    // claims while its stored mask still names them: a worse outcome, and one
    // that depends on portConfigs[] iteration order.
    for (unsigned i = 0; i < ARRAYLEN(serialConfig()->portConfigs); i++) {
        const serialPortConfig_t *port = &serialConfig()->portConfigs[i];
        (void)applyFunctionMask(port->identifier, port->functionMask, false);
    }
}
