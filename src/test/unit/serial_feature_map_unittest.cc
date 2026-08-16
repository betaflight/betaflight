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
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/utils.h"

    #include "drivers/vtx_common.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/msp.h"
    #include "pg/gps.h"
    #include "pg/rx.h"
    #include "pg/rcdevice.h"
    #include "pg/gimbal.h"

    #include "blackbox/blackbox.h"

    #include "sensors/esc_sensor.h"
    #include "sensors/opticalflow.h"
    #include "sensors/rangefinder.h"

    #include "osd/osd.h"

    #include "io/serial.h"
    #include "io/serial_feature_map.h"
    #include "io/vtx.h"

    #include "telemetry/telemetry.h"

    // PG instances for feature configs read by the synthesizer.
    PG_REGISTER(mspConfig_t, mspConfig, PG_MSP_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(escSensorConfig_t, escSensorConfig, PG_ESC_SENSOR_CONFIG, 0);
    PG_REGISTER(rcdeviceConfig_t, rcdeviceConfig, PG_RCDEVICE_CONFIG, 0);
    PG_REGISTER(gimbalTrackConfig_t, gimbalTrackConfig, PG_GIMBAL_TRACK_CONFIG, 0);
    PG_REGISTER(vtxSettingsConfig_t, vtxSettingsConfig, PG_VTX_SETTINGS_CONFIG, 0);
    PG_REGISTER(rangefinderConfig_t, rangefinderConfig, PG_RANGEFINDER_CONFIG, 0);
    PG_REGISTER(opticalflowConfig_t, opticalflowConfig, PG_OPTICALFLOW_CONFIG, 0);
    PG_REGISTER(osdConfig_t, osdConfig, PG_OSD_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

    const serialPortIdentifier_e serialPortIdentifiers[SERIAL_PORT_COUNT] = {
        SERIAL_PORT_USB_VCP,
        SERIAL_PORT_USART1,
        SERIAL_PORT_USART2,
        SERIAL_PORT_USART3,
        SERIAL_PORT_UART4,
        SERIAL_PORT_UART5,
        SERIAL_PORT_SOFTSERIAL1,
        SERIAL_PORT_SOFTSERIAL2,
    };

    // Which ports clash is serial.c's judgement and is not linked here; naming
    // them directly keeps these tests on what the drop itself does.
    serialPortIdentifier_e conflictingPort = SERIAL_PORT_NONE;

    bool serialPortFunctionsConflict(serialPortIdentifier_e identifier)
    {
        return identifier == conflictingPort;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

// Reset every PG under test to an "unassigned" baseline.
void resetAllConfigs(void)
{
    memset(mspConfigMutable(), 0, sizeof(*mspConfigMutable()));
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        mspConfigMutable()->msp_uart[i] = SERIAL_PORT_NONE;
        mspConfigMutable()->msp_baud[i] = BAUD_115200;
    }

    memset(gpsConfigMutable(), 0, sizeof(*gpsConfigMutable()));
    gpsConfigMutable()->gps_uart = SERIAL_PORT_NONE;
    gpsConfigMutable()->gps_baud = BAUD_57600;

    memset(rxConfigMutable(), 0, sizeof(*rxConfigMutable()));
    rxConfigMutable()->rx_uart = SERIAL_PORT_NONE;

    memset(blackboxConfigMutable(), 0, sizeof(*blackboxConfigMutable()));
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_NONE;
    blackboxConfigMutable()->blackbox_baud = BAUD_115200;

    memset(escSensorConfigMutable(), 0, sizeof(*escSensorConfigMutable()));
    escSensorConfigMutable()->esc_sensor_uart = SERIAL_PORT_NONE;

    memset(rcdeviceConfigMutable(), 0, sizeof(*rcdeviceConfigMutable()));
    rcdeviceConfigMutable()->rcdevice_uart = SERIAL_PORT_NONE;

    memset(gimbalTrackConfigMutable(), 0, sizeof(*gimbalTrackConfigMutable()));
    gimbalTrackConfigMutable()->gimbal_uart = SERIAL_PORT_NONE;

    memset(vtxSettingsConfigMutable(), 0, sizeof(*vtxSettingsConfigMutable()));
    vtxSettingsConfigMutable()->vtx_uart = SERIAL_PORT_NONE;
    vtxSettingsConfigMutable()->vtx_type = VTXDEV_UNSUPPORTED;

    memset(rangefinderConfigMutable(), 0, sizeof(*rangefinderConfigMutable()));
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_NONE;
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;

    memset(opticalflowConfigMutable(), 0, sizeof(*opticalflowConfigMutable()));
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_NONE;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_NONE;

    memset(osdConfigMutable(), 0, sizeof(*osdConfigMutable()));
    osdConfigMutable()->osd_uart = SERIAL_PORT_NONE;
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_NONE;
    osdConfigMutable()->osd_custom_text_baud = BAUD_AUTO;

    memset(telemetryConfigMutable(), 0, sizeof(*telemetryConfigMutable()));
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        telemetryConfigMutable()->providers[i].protocol = TELEMETRY_PROTOCOL_NONE;
        telemetryConfigMutable()->providers[i].uart = SERIAL_PORT_NONE;
        telemetryConfigMutable()->providers[i].baud = BAUD_AUTO;
    }

    memset(serialConfigMutable(), 0, sizeof(*serialConfigMutable()));
}

} // namespace

TEST(SerialFeatureMap, SynthesizesZeroWhenNothingAssigned)
{
    resetAllConfigs();
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USB_VCP));
}

TEST(SerialFeatureMap, SingleBitFeatures)
{
    resetAllConfigs();
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART1;
    rxConfigMutable()->rx_uart = SERIAL_PORT_USART3;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_USART6;
    escSensorConfigMutable()->esc_sensor_uart = SERIAL_PORT_USART2;

    EXPECT_EQ(FUNCTION_GPS, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ(FUNCTION_RX_SERIAL, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
    EXPECT_EQ(FUNCTION_BLACKBOX, serialSynthesizeFunctionMask(SERIAL_PORT_USART6));
    EXPECT_EQ(FUNCTION_ESC_SENSOR, serialSynthesizeFunctionMask(SERIAL_PORT_USART2));
    // Unassigned port stays empty.
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));
}

TEST(SerialFeatureMap, MspMultiSlot)
{
    resetAllConfigs();
    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USB_VCP;
    mspConfigMutable()->msp_uart[1] = SERIAL_PORT_USART3;

    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USB_VCP));
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
}

TEST(SerialFeatureMap, TelemetryProviderSlots)
{
    resetAllConfigs();
    telemetryConfigMutable()->providers[0].protocol = TELEMETRY_PROTOCOL_SMARTPORT;
    telemetryConfigMutable()->providers[0].uart = SERIAL_PORT_USART3;
    telemetryConfigMutable()->providers[1].protocol = TELEMETRY_PROTOCOL_MAVLINK;
    telemetryConfigMutable()->providers[1].uart = SERIAL_PORT_USART6;

    EXPECT_EQ(FUNCTION_TELEMETRY_SMARTPORT, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
    EXPECT_EQ(FUNCTION_TELEMETRY_MAVLINK, serialSynthesizeFunctionMask(SERIAL_PORT_USART6));
}

TEST(SerialFeatureMap, VtxCollapseByType)
{
    resetAllConfigs();
    vtxSettingsConfigMutable()->vtx_uart = SERIAL_PORT_UART4;

    vtxSettingsConfigMutable()->vtx_type = VTXDEV_SMARTAUDIO;
    EXPECT_EQ(FUNCTION_VTX_SMARTAUDIO, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));

    vtxSettingsConfigMutable()->vtx_type = VTXDEV_TRAMP;
    EXPECT_EQ(FUNCTION_VTX_TRAMP, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));

    vtxSettingsConfigMutable()->vtx_type = VTXDEV_MSP;
    EXPECT_EQ(FUNCTION_VTX_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));

    // Unsupported VTX type means the port has no VTX function bit.
    vtxSettingsConfigMutable()->vtx_type = VTXDEV_UNSUPPORTED;
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));
}

TEST(SerialFeatureMap, RangefinderIndependentOfHardwareSelection)
{
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART5;

    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));

    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NOOPLOOP_F2;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));

    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));

    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_NONE;
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));
}

TEST(SerialFeatureMap, OpticalflowClaimsTheLidarBit)
{
    resetAllConfigs();
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_UART4;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));

    // One module answering as both sensors declares the port twice, and the
    // shared bit has to read the same as either sensor alone.
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));

    // Separate modules light the bit on both ports independently.
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_UART5;
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));
}

TEST(SerialFeatureMap, TransportFollowsHardwareSelection)
{
    // Native serial devices open the port from their own driver.
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_NONE));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_TFMINI));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_TF02));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_TFNOVA));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_NOOPLOOP_F2));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_NOOPLOOP_F2MINI));
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_UPT1));
    // Pin-driven, no UART at all.
    EXPECT_FALSE(rangefinderTypeUsesMsp(RANGEFINDER_HCSR04));

    // Every MT variant reports over MSP.
    EXPECT_TRUE(rangefinderTypeUsesMsp(RANGEFINDER_MTF01));
    EXPECT_TRUE(rangefinderTypeUsesMsp(RANGEFINDER_MTF02));
    EXPECT_TRUE(rangefinderTypeUsesMsp(RANGEFINDER_MTF01P));
    EXPECT_TRUE(rangefinderTypeUsesMsp(RANGEFINDER_MTF02P));

    EXPECT_FALSE(opticalflowTypeUsesMsp(OPTICALFLOW_NONE));
    EXPECT_FALSE(opticalflowTypeUsesMsp(OPTICALFLOW_UPT1));
    EXPECT_TRUE(opticalflowTypeUsesMsp(OPTICALFLOW_MT));
}

TEST(SerialFeatureMap, ImpliedMspPortNeedsBothHardwareAndPort)
{
    serialPortIdentifier_e ports[IMPLIED_MSP_SENSOR_PORT_COUNT];

    resetAllConfigs();
    EXPECT_EQ(0u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));

    // A declared port on its own says nothing about the transport.
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    EXPECT_EQ(0u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));

    // Nor does the hardware on its own, with no port to open.
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_NONE;
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;
    EXPECT_EQ(0u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));

    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    ASSERT_EQ(1u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));
    EXPECT_EQ(SERIAL_PORT_UART4, ports[0]);

    // A native serial sensor claims its UART in sensorsAutodetect() instead.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
    EXPECT_EQ(0u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));
}

TEST(SerialFeatureMap, ImpliedMspPortCountsOneModuleOnce)
{
    serialPortIdentifier_e ports[IMPLIED_MSP_SENSOR_PORT_COUNT];

    // Both sensors of one MT module land on a single declared port.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_UART4;

    ASSERT_EQ(1u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));
    EXPECT_EQ(SERIAL_PORT_UART4, ports[0]);

    // Two modules on separate UARTs need one port each.
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_UART5;
    ASSERT_EQ(2u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));
    EXPECT_EQ(SERIAL_PORT_UART4, ports[0]);
    EXPECT_EQ(SERIAL_PORT_UART5, ports[1]);

    // A native rangefinder alongside an MSP flow module leaves only the flow.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TF02;
    ASSERT_EQ(1u, serialImpliedMspPorts(ports, ARRAYLEN(ports)));
    EXPECT_EQ(SERIAL_PORT_UART5, ports[0]);
}

TEST(SerialFeatureMap, ImpliedMspPortsRespectsCallerCapacity)
{
    serialPortIdentifier_e ports[IMPLIED_MSP_SENSOR_PORT_COUNT];

    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF01;
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_UART5;

    EXPECT_EQ(0u, serialImpliedMspPorts(ports, 0));
    EXPECT_EQ(1u, serialImpliedMspPorts(ports, 1));
}

TEST(SerialFeatureMap, OsdCollapseByDisplayPortDevice)
{
    resetAllConfigs();
    osdConfigMutable()->osd_uart = SERIAL_PORT_USART1;

    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_FRSKYOSD;
    EXPECT_EQ(FUNCTION_FRSKY_OSD, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));

    // MSP-displayport uses an existing MSP port, so osd_uart contributes no bit.
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
}

TEST(SerialFeatureMap, SharedPortCombinesBits)
{
    resetAllConfigs();
    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USART3;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_USART3;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART3;
    telemetryConfigMutable()->providers[0].protocol = TELEMETRY_PROTOCOL_SMARTPORT;
    telemetryConfigMutable()->providers[0].uart = SERIAL_PORT_USART3;

    EXPECT_EQ((uint32_t)(FUNCTION_MSP | FUNCTION_BLACKBOX | FUNCTION_GPS | FUNCTION_TELEMETRY_SMARTPORT),
              serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
}

TEST(SerialFeatureMap, RcdeviceGimbalAndOsdCustomText)
{
    resetAllConfigs();
    rcdeviceConfigMutable()->rcdevice_uart = SERIAL_PORT_USART1;
    gimbalTrackConfigMutable()->gimbal_uart = SERIAL_PORT_USART2;
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_UART4;

    EXPECT_EQ(FUNCTION_RCDEVICE, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ(FUNCTION_GIMBAL, serialSynthesizeFunctionMask(SERIAL_PORT_USART2));
    EXPECT_EQ(FUNCTION_OSD_CUSTOM_TEXT, serialSynthesizeFunctionMask(SERIAL_PORT_UART4));
}

TEST(SerialFeatureMap, MspOnEverySlot)
{
    resetAllConfigs();
    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USB_VCP;
    mspConfigMutable()->msp_uart[1] = SERIAL_PORT_USART1;
    mspConfigMutable()->msp_uart[2] = SERIAL_PORT_USART3;

    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USB_VCP));
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART6));
}

TEST(SerialFeatureMap, AllTelemetryProtocols)
{
    resetAllConfigs();

    const struct {
        uint8_t protocol;
        serialPortIdentifier_e port;
        uint32_t function;
    } cases[] = {
        { TELEMETRY_PROTOCOL_FRSKY_HUB, SERIAL_PORT_USART1, FUNCTION_TELEMETRY_FRSKY_HUB },
        { TELEMETRY_PROTOCOL_HOTT,      SERIAL_PORT_USART2, FUNCTION_TELEMETRY_HOTT },
        { TELEMETRY_PROTOCOL_LTM,       SERIAL_PORT_USART3, FUNCTION_TELEMETRY_LTM },
        { TELEMETRY_PROTOCOL_SMARTPORT, SERIAL_PORT_UART4,  FUNCTION_TELEMETRY_SMARTPORT },
        { TELEMETRY_PROTOCOL_MAVLINK,   SERIAL_PORT_UART5,  FUNCTION_TELEMETRY_MAVLINK },
        { TELEMETRY_PROTOCOL_IBUS,      SERIAL_PORT_USART6, FUNCTION_TELEMETRY_IBUS },
    };

    // More protocols than slots, so exercise each one on its own.
    for (unsigned i = 0; i < ARRAYLEN(cases); i++) {
        resetAllConfigs();
        telemetryConfigMutable()->providers[0].protocol = cases[i].protocol;
        telemetryConfigMutable()->providers[0].uart = cases[i].port;

        EXPECT_EQ(cases[i].function, serialSynthesizeFunctionMask(cases[i].port));
    }
}

TEST(SerialFeatureMap, ResetClearsEveryClaimAndRestoresMsp)
{
    resetAllConfigs();

    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USART1;
    mspConfigMutable()->msp_uart[1] = SERIAL_PORT_USART2;
    mspConfigMutable()->msp_uart[2] = SERIAL_PORT_USART3;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_UART4;
    rxConfigMutable()->rx_uart = SERIAL_PORT_UART5;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_SOFTSERIAL1;
    escSensorConfigMutable()->esc_sensor_uart = SERIAL_PORT_SOFTSERIAL2;
    rcdeviceConfigMutable()->rcdevice_uart = SERIAL_PORT_USART1;
    gimbalTrackConfigMutable()->gimbal_uart = SERIAL_PORT_USART2;
    vtxSettingsConfigMutable()->vtx_uart = SERIAL_PORT_USART3;
    vtxSettingsConfigMutable()->vtx_type = VTXDEV_TRAMP;
    rangefinderConfigMutable()->rangefinder_uart = SERIAL_PORT_UART4;
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFNOVA;
    opticalflowConfigMutable()->opticalflow_uart = SERIAL_PORT_SOFTSERIAL1;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    osdConfigMutable()->osd_uart = SERIAL_PORT_UART5;
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_FRSKYOSD;
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_SOFTSERIAL1;
    telemetryConfigMutable()->providers[0].protocol = TELEMETRY_PROTOCOL_SMARTPORT;
    telemetryConfigMutable()->providers[0].uart = SERIAL_PORT_SOFTSERIAL2;
    telemetryConfigMutable()->providers[1].protocol = TELEMETRY_PROTOCOL_MAVLINK;
    telemetryConfigMutable()->providers[1].uart = SERIAL_PORT_USB_VCP;

    serialResetFeatureAssignments();

    EXPECT_EQ(serialPortIdentifiers[0], mspConfig()->msp_uart[0]);
    for (unsigned i = 1; i < MAX_MSP_PORT_COUNT; i++) {
        EXPECT_EQ(SERIAL_PORT_NONE, mspConfig()->msp_uart[i]);
    }
    EXPECT_EQ(SERIAL_PORT_NONE, gpsConfig()->gps_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, rxConfig()->rx_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, blackboxConfig()->blackbox_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, escSensorConfig()->esc_sensor_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, rcdeviceConfig()->rcdevice_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, gimbalTrackConfig()->gimbal_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, vtxSettingsConfig()->vtx_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, rangefinderConfig()->rangefinder_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, opticalflowConfig()->opticalflow_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, osdConfig()->osd_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, osdConfig()->osd_custom_text_uart);
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        EXPECT_EQ(SERIAL_PORT_NONE, telemetryConfig()->providers[i].uart);
        EXPECT_EQ(TELEMETRY_PROTOCOL_NONE, telemetryConfig()->providers[i].protocol);
    }

    // Hardware/protocol selectors are not port claims and must survive.
    EXPECT_EQ(VTXDEV_TRAMP, vtxSettingsConfig()->vtx_type);
    EXPECT_EQ(RANGEFINDER_TFNOVA, rangefinderConfig()->rangefinder_hardware);
    EXPECT_EQ(OPTICALFLOW_MT, opticalflowConfig()->opticalflow_hardware);
    EXPECT_EQ(OSD_DISPLAYPORT_DEVICE_FRSKYOSD, osdConfig()->displayPortDevice);

    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(serialPortIdentifiers[0]));
    for (unsigned i = 1; i < SERIAL_PORT_COUNT; i++) {
        EXPECT_EQ(0u, serialSynthesizeFunctionMask(serialPortIdentifiers[i]));
    }
}

TEST(SerialFeatureMap, DropConflictingLeavesInnocentPortsAlone)
{
    resetAllConfigs();

    // The board-wide reset this replaces would have taken all four assignments
    // to settle the one clash on USART1.
    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USART1;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART1;
    rxConfigMutable()->rx_uart = SERIAL_PORT_USART3;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_UART4;
    escSensorConfigMutable()->esc_sensor_uart = SERIAL_PORT_UART5;

    conflictingPort = SERIAL_PORT_USART1;
    serialDropConflictingAssignments();
    conflictingPort = SERIAL_PORT_NONE;

    EXPECT_EQ(SERIAL_PORT_NONE, gpsConfig()->gps_uart);
    EXPECT_EQ(SERIAL_PORT_USART3, rxConfig()->rx_uart);
    EXPECT_EQ(SERIAL_PORT_UART4, blackboxConfig()->blackbox_uart);
    EXPECT_EQ(SERIAL_PORT_UART5, escSensorConfig()->esc_sensor_uart);
}

TEST(SerialFeatureMap, DropConflictingKeepsMspSoTheBoardStaysReachable)
{
    resetAllConfigs();

    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USART1;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART1;

    conflictingPort = SERIAL_PORT_USART1;
    serialDropConflictingAssignments();
    conflictingPort = SERIAL_PORT_NONE;

    EXPECT_EQ(SERIAL_PORT_USART1, mspConfig()->msp_uart[0]);
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
}

TEST(SerialFeatureMap, DropConflictingTouchesNothingWhenEveryPortIsHappy)
{
    resetAllConfigs();

    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_USART1;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_USART1;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART3;

    serialDropConflictingAssignments();

    EXPECT_EQ(SERIAL_PORT_USART1, mspConfig()->msp_uart[0]);
    EXPECT_EQ(SERIAL_PORT_USART1, blackboxConfig()->blackbox_uart);
    EXPECT_EQ(SERIAL_PORT_USART3, gpsConfig()->gps_uart);
}

TEST(SerialFeatureMap, UnclaimedPortReportsClassDefaults)
{
    resetAllConfigs();

    // Matches the legacy pgResetFn_serialConfig values, so an untouched config
    // still round-trips through `dump` and MSP unchanged.
    EXPECT_EQ(BAUD_115200, serialDefaultPortBaud(SERIAL_BAUD_MSP));
    EXPECT_EQ(BAUD_57600,  serialDefaultPortBaud(SERIAL_BAUD_GPS));
    EXPECT_EQ(BAUD_AUTO,   serialDefaultPortBaud(SERIAL_BAUD_TELEMETRY));
    EXPECT_EQ(BAUD_115200, serialDefaultPortBaud(SERIAL_BAUD_BLACKBOX));

    for (unsigned c = 0; c < SERIAL_BAUD_CLASS_COUNT; c++) {
        EXPECT_EQ(serialDefaultPortBaud((serialBaudClass_e)c),
                  serialSynthesizePortBaud(SERIAL_PORT_UART4, (serialBaudClass_e)c));
    }
}

TEST(SerialFeatureMap, BaudReadsFromOwningFeature)
{
    resetAllConfigs();

    gpsConfigMutable()->gps_uart = SERIAL_PORT_UART4;
    gpsConfigMutable()->gps_baud = BAUD_115200;
    EXPECT_EQ(BAUD_115200, serialSynthesizePortBaud(SERIAL_PORT_UART4, SERIAL_BAUD_GPS));

    // A port the feature does not own keeps reporting the class default.
    EXPECT_EQ(BAUD_57600, serialSynthesizePortBaud(SERIAL_PORT_UART5, SERIAL_BAUD_GPS));

    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_UART5;
    blackboxConfigMutable()->blackbox_baud = BAUD_230400;
    EXPECT_EQ(BAUD_230400, serialSynthesizePortBaud(SERIAL_PORT_UART5, SERIAL_BAUD_BLACKBOX));
}

TEST(SerialFeatureMap, MspBaudIsPerSlot)
{
    resetAllConfigs();

    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_UART4;
    mspConfigMutable()->msp_baud[0] = BAUD_115200;
    mspConfigMutable()->msp_uart[1] = SERIAL_PORT_UART5;
    mspConfigMutable()->msp_baud[1] = BAUD_500000;

    EXPECT_EQ(BAUD_115200, serialSynthesizePortBaud(SERIAL_PORT_UART4, SERIAL_BAUD_MSP));
    EXPECT_EQ(BAUD_500000, serialSynthesizePortBaud(SERIAL_PORT_UART5, SERIAL_BAUD_MSP));

    // A port held by no slot falls back to the class default.
    EXPECT_EQ(BAUD_115200, serialSynthesizePortBaud(SERIAL_PORT_USART1, SERIAL_BAUD_MSP));
}

TEST(SerialFeatureMap, TelemetryBaudIsPerProviderSlot)
{
    resetAllConfigs();

    telemetryConfigMutable()->providers[0].protocol = TELEMETRY_PROTOCOL_LTM;
    telemetryConfigMutable()->providers[0].uart = SERIAL_PORT_UART4;
    telemetryConfigMutable()->providers[0].baud = BAUD_19200;
    telemetryConfigMutable()->providers[1].protocol = TELEMETRY_PROTOCOL_MAVLINK;
    telemetryConfigMutable()->providers[1].uart = SERIAL_PORT_UART5;
    telemetryConfigMutable()->providers[1].baud = BAUD_57600;

    EXPECT_EQ(BAUD_19200, serialSynthesizePortBaud(SERIAL_PORT_UART4, SERIAL_BAUD_TELEMETRY));
    EXPECT_EQ(BAUD_57600, serialSynthesizePortBaud(SERIAL_PORT_UART5, SERIAL_BAUD_TELEMETRY));
}

TEST(SerialFeatureMap, OsdCustomTextBaudRidesTheTelemetryClass)
{
    resetAllConfigs();

    // It is not telemetry, but it shared the port's telemetry baud in the
    // legacy layout and must keep reporting through that class.
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_UART6;
    osdConfigMutable()->osd_custom_text_baud = BAUD_115200;

    EXPECT_EQ(BAUD_115200, serialSynthesizePortBaud(SERIAL_PORT_UART6, SERIAL_BAUD_TELEMETRY));
    EXPECT_EQ(BAUD_AUTO, serialSynthesizePortBaud(SERIAL_PORT_UART7, SERIAL_BAUD_TELEMETRY));
}

TEST(SerialFeatureMap, BaudClassesAreIndependentOnASharedPort)
{
    resetAllConfigs();

    mspConfigMutable()->msp_uart[0] = SERIAL_PORT_UART4;
    mspConfigMutable()->msp_baud[0] = BAUD_500000;
    gpsConfigMutable()->gps_uart = SERIAL_PORT_UART4;
    gpsConfigMutable()->gps_baud = BAUD_9600;
    telemetryConfigMutable()->providers[0].protocol = TELEMETRY_PROTOCOL_LTM;
    telemetryConfigMutable()->providers[0].uart = SERIAL_PORT_UART4;
    telemetryConfigMutable()->providers[0].baud = BAUD_38400;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_UART4;
    blackboxConfigMutable()->blackbox_baud = BAUD_230400;

    const uint8_t expected[SERIAL_BAUD_CLASS_COUNT] = {
        BAUD_500000,  // msp
        BAUD_9600,    // gps
        BAUD_38400,   // telemetry
        BAUD_230400,  // blackbox
    };

    for (unsigned c = 0; c < SERIAL_BAUD_CLASS_COUNT; c++) {
        EXPECT_EQ(expected[c], serialSynthesizePortBaud(SERIAL_PORT_UART4, (serialBaudClass_e)c));
    }
}
