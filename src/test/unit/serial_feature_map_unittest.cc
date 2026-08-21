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

    // PG instances for feature configs read by the synthesizer/decomposer.
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
    }

    memset(gpsConfigMutable(), 0, sizeof(*gpsConfigMutable()));
    gpsConfigMutable()->gps_uart = SERIAL_PORT_NONE;

    memset(rxConfigMutable(), 0, sizeof(*rxConfigMutable()));
    rxConfigMutable()->rx_uart = SERIAL_PORT_NONE;

    memset(blackboxConfigMutable(), 0, sizeof(*blackboxConfigMutable()));
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_NONE;

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
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_NONE;

    memset(osdConfigMutable(), 0, sizeof(*osdConfigMutable()));
    osdConfigMutable()->osd_uart = SERIAL_PORT_NONE;
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_NONE;

    memset(telemetryConfigMutable(), 0, sizeof(*telemetryConfigMutable()));
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        telemetryConfigMutable()->providers[i].protocol = TELEMETRY_PROTOCOL_NONE;
        telemetryConfigMutable()->providers[i].uart = SERIAL_PORT_NONE;
    }

    memset(serialConfigMutable(), 0, sizeof(*serialConfigMutable()));
    for (unsigned i = 0; i < SERIAL_PORT_COUNT; i++) {
        serialConfigMutable()->portConfigs[i].identifier = SERIAL_PORT_NONE;
    }
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

    EXPECT_EQ((uint32_t)(FUNCTION_MSP | FUNCTION_BLACKBOX),
              serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
}

TEST(SerialFeatureMap, DecomposeAndSynthesizeRoundTrip)
{
    resetAllConfigs();

    const serialPortIdentifier_e gpsPort = SERIAL_PORT_USART1;
    const serialPortIdentifier_e rxPort = SERIAL_PORT_USART3;
    const serialPortIdentifier_e mspPort = SERIAL_PORT_USB_VCP;
    const serialPortIdentifier_e tlmPort = SERIAL_PORT_USART6;

    EXPECT_TRUE(serialApplyFunctionMask(gpsPort, FUNCTION_GPS));
    EXPECT_TRUE(serialApplyFunctionMask(rxPort, FUNCTION_RX_SERIAL));
    EXPECT_TRUE(serialApplyFunctionMask(mspPort, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(tlmPort, FUNCTION_TELEMETRY_SMARTPORT));

    EXPECT_EQ(FUNCTION_GPS, serialSynthesizeFunctionMask(gpsPort));
    EXPECT_EQ(FUNCTION_RX_SERIAL, serialSynthesizeFunctionMask(rxPort));
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(mspPort));
    EXPECT_EQ(FUNCTION_TELEMETRY_SMARTPORT, serialSynthesizeFunctionMask(tlmPort));
}

TEST(SerialFeatureMap, DecomposeReassignClearsOldPort)
{
    resetAllConfigs();
    // First put GPS on UART1.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_GPS));
    EXPECT_EQ(FUNCTION_GPS, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));

    // Rewrite UART1 with only FUNCTION_RX_SERIAL — GPS must leave the port.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_RX_SERIAL));
    EXPECT_EQ(FUNCTION_RX_SERIAL, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ(SERIAL_PORT_NONE, gpsConfig()->gps_uart);
}

TEST(SerialFeatureMap, DecomposeMspFillsAcrossSlots)
{
    resetAllConfigs();
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));

    // Both ports should now produce FUNCTION_MSP.
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USB_VCP));
    EXPECT_EQ(FUNCTION_MSP, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
}

TEST(SerialFeatureMap, DecomposeRejectsMspOverflow)
{
    resetAllConfigs();
    // Fill every MSP slot.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
    // One more should fail because MAX_MSP_PORT_COUNT is exhausted.
    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART6, FUNCTION_MSP));
}

TEST(SerialFeatureMap, FailedApplyLeavesPriorStateIntact)
{
    resetAllConfigs();
    // Set up a valid claim, then attempt an invalid apply on the same port.
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART1;
    blackboxConfigMutable()->blackbox_uart = SERIAL_PORT_USART1;

    // Two VTX bits on one port is structurally invalid and must fail.
    EXPECT_FALSE(serialApplyFunctionMask(
        SERIAL_PORT_USART1, FUNCTION_VTX_SMARTAUDIO | FUNCTION_VTX_TRAMP));

    // Prior feature claims for the port survive — validation happens before
    // any PG mutation, so a rejected mask is atomic from the caller's view.
    EXPECT_EQ(SERIAL_PORT_USART1, gpsConfig()->gps_uart);
    EXPECT_EQ(SERIAL_PORT_USART1, blackboxConfig()->blackbox_uart);
    EXPECT_EQ(SERIAL_PORT_NONE, vtxSettingsConfig()->vtx_uart);
    EXPECT_EQ(VTXDEV_UNSUPPORTED, vtxSettingsConfig()->vtx_type);
}

TEST(SerialFeatureMap, DecomposeAcceptsAllTelemetryProtocols)
{
    resetAllConfigs();
    // Six protocol bits across six distinct ports must all fit.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_TELEMETRY_FRSKY_HUB));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_TELEMETRY_HOTT));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_TELEMETRY_LTM));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART4,  FUNCTION_TELEMETRY_SMARTPORT));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5,  FUNCTION_TELEMETRY_MAVLINK));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART6, FUNCTION_TELEMETRY_IBUS));
}

TEST(SerialFeatureMap, RangefinderPreservesHardwareSelection)
{
    resetAllConfigs();
    // The port assignment and the driver choice are orthogonal: applying the
    // mask must never disturb the user's model selection.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFNOVA;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_EQ(SERIAL_PORT_UART5, rangefinderConfig()->rangefinder_uart);
    EXPECT_EQ(RANGEFINDER_TFNOVA, rangefinderConfig()->rangefinder_hardware);

    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NOOPLOOP_F2MINI;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_EQ(RANGEFINDER_NOOPLOOP_F2MINI, rangefinderConfig()->rangefinder_hardware);

    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_EQ(RANGEFINDER_NONE, rangefinderConfig()->rangefinder_hardware);
}

TEST(SerialFeatureMap, SensorPortTransportFollowsHardwareSelection)
{
    resetAllConfigs();
    EXPECT_FALSE(serialSensorPortUsesMsp());

    // Native serial rangefinders open the port themselves.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
    EXPECT_FALSE(serialSensorPortUsesMsp());
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NOOPLOOP_F2;
    EXPECT_FALSE(serialSensorPortUsesMsp());
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_UPT1;
    EXPECT_FALSE(serialSensorPortUsesMsp());
    // Pin-driven, no UART at all.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_HCSR04;
    EXPECT_FALSE(serialSensorPortUsesMsp());

    // Every MT variant reports over MSP.
    for (const uint8_t hardware : {RANGEFINDER_MTF01, RANGEFINDER_MTF02,
                                   RANGEFINDER_MTF01P, RANGEFINDER_MTF02P}) {
        rangefinderConfigMutable()->rangefinder_hardware = hardware;
        EXPECT_TRUE(serialSensorPortUsesMsp());
    }

    // An MT optical flow sensor with no rangefinder configured still needs one.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_NONE;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    EXPECT_TRUE(serialSensorPortUsesMsp());

    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_UPT1;
    EXPECT_FALSE(serialSensorPortUsesMsp());
}

TEST(SerialFeatureMap, NativeSerialSensorWinsOverAnMspOneInBothOrders)
{
    // One declared port carries one module, so a pair of selections that
    // disagree is not a real wiring.  The native driver takes the port during
    // sensorsAutodetect() whichever way round it is configured, so no MSP slot
    // may be reserved for it.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF01;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_UPT1;
    EXPECT_FALSE(serialSensorPortUsesMsp());

    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_UPT1;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    EXPECT_FALSE(serialSensorPortUsesMsp());

    // HCSR04 is pin-driven and claims no UART, so it does not veto anything.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_HCSR04;
    opticalflowConfigMutable()->opticalflow_hardware = OPTICALFLOW_MT;
    EXPECT_TRUE(serialSensorPortUsesMsp());
}

TEST(SerialFeatureMap, SensorPortAssignedFirstStillHoldsItsMspSlot)
{
    // The order that used to defeat the budget check: the sensor claims no
    // msp_uart[] slot, so counting only the mask being applied let every
    // remaining slot be handed out afterwards and left the sensor silent.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF01;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));

    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    // The third would take the entry the sensor needs at boot.
    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));

    // The sensor keeps its port, and the refused write changed nothing.
    EXPECT_EQ(SERIAL_PORT_UART5, rangefinderConfig()->rangefinder_uart);
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));

    // A native serial sensor reserves nothing, so the same board fits three.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
}

TEST(SerialFeatureMap, AnOverBudgetConfigStaysEditable)
{
    // rangefinder_hardware is not written through canApplyFunctionMask, so a
    // config can go over budget behind its back: assign the sensor port while
    // the sensor is still NONE, then pick an MT module in the Sensors tab.
    resetAllConfigs();
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;

    // Every Ports-tab save starts with the USB VCP, which must carry MSP.
    // Refusing it because the total is already over budget would freeze the
    // tab: the later record that frees a slot is never reached, and firmware
    // rejects the whole write.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));

    // The move that digs the user out is allowed...
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_NONE));
    // ...and once back inside the budget the limit bites again as before.
    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
}

TEST(SerialFeatureMap, AnOverBudgetConfigStillRefusesToGetWorse)
{
    resetAllConfigs();
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;

    // Holding the count is allowed; adding to it is not.
    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART6, FUNCTION_MSP));
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART6));
}

TEST(SerialFeatureMap, ClearingTheSensorPortReleasesItsReservation)
{
    // Removing FUNCTION_LIDAR from the port that holds it must not go on
    // reserving a slot for a sensor that no longer has a port.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));

    // Hand the sensor's own port back, then the freed budget is spendable.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_NONE));
    EXPECT_EQ(SERIAL_PORT_NONE, rangefinderConfig()->rangefinder_uart);
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
}

TEST(SerialFeatureMap, BackfillDoesNotReserveForTheSensor)
{
    // An over-budget stored config is well-formed and boots today with a deaf
    // sensor.  Migration must carry it across intact rather than reject a port
    // and drop its feature claims.
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF01;

    serialConfigMutable()->portConfigs[0].identifier = SERIAL_PORT_USB_VCP;
    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_MSP;
    serialConfigMutable()->portConfigs[1].identifier = SERIAL_PORT_USART1;
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_MSP;
    serialConfigMutable()->portConfigs[2].identifier = SERIAL_PORT_USART3;
    serialConfigMutable()->portConfigs[2].functionMask = FUNCTION_MSP;
    serialConfigMutable()->portConfigs[3].identifier = SERIAL_PORT_UART5;
    serialConfigMutable()->portConfigs[3].functionMask = FUNCTION_LIDAR;

    serialBackfillFeatureFields();

    EXPECT_EQ(SERIAL_PORT_USB_VCP, mspConfig()->msp_uart[0]);
    EXPECT_EQ(SERIAL_PORT_USART1, mspConfig()->msp_uart[1]);
    EXPECT_EQ(SERIAL_PORT_USART3, mspConfig()->msp_uart[2]);
    EXPECT_EQ(SERIAL_PORT_UART5, rangefinderConfig()->rangefinder_uart);
}

TEST(SerialFeatureMap, SensorPortOnMspTransportNeedsAnMspSlotFree)
{
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02;
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        EXPECT_TRUE(serialApplyFunctionMask((serialPortIdentifier_e)(SERIAL_PORT_USART1 + i), FUNCTION_MSP));
    }

    // The sensor port claims no msp_uart[] slot, but one is opened for it at
    // boot out of the same MAX_MSP_PORT_COUNT budget - with none left the
    // module would be silent, so the assignment is refused rather than taken.
    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_EQ(SERIAL_PORT_NONE, rangefinderConfig()->rangefinder_uart);

    // A native serial sensor on the same board needs nothing from that budget.
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_TFMINI;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));
    EXPECT_EQ(SERIAL_PORT_UART5, rangefinderConfig()->rangefinder_uart);
}

TEST(SerialFeatureMap, SensorPortSharedWithMspCountsOnce)
{
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF01;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));

    // One slot left, and a port naming both wants exactly one: the explicit
    // MSP claim is the port the sensor would have been given anyway.
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR | FUNCTION_MSP));
    EXPECT_EQ(SERIAL_PORT_UART5, rangefinderConfig()->rangefinder_uart);
    EXPECT_EQ((uint32_t)(FUNCTION_LIDAR | FUNCTION_MSP),
              serialSynthesizeFunctionMask(SERIAL_PORT_UART5));
}

TEST(SerialFeatureMap, SensorPortDoesNotClaimAnMspSlot)
{
    resetAllConfigs();
    rangefinderConfigMutable()->rangefinder_hardware = RANGEFINDER_MTF02P;
    EXPECT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_LIDAR));

    // The implied MSP port is resolved at allocation time from the sensor's
    // own function, never recorded here - a stored slot could not follow a
    // later rangefinder_hardware change.
    EXPECT_EQ(FUNCTION_LIDAR, serialSynthesizeFunctionMask(SERIAL_PORT_UART5));
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        EXPECT_EQ(SERIAL_PORT_NONE, mspConfig()->msp_uart[i]);
    }
}

TEST(SerialFeatureMap, BackfillRehydratesFromLegacyMask)
{
    resetAllConfigs();
    // Simulate an EEPROM load: populate the legacy functionMask and
    // identifier on a couple of port configs.
    serialConfigMutable()->portConfigs[0].identifier = SERIAL_PORT_USART1;
    serialConfigMutable()->portConfigs[0].functionMask = FUNCTION_GPS;
    serialConfigMutable()->portConfigs[1].identifier = SERIAL_PORT_USART3;
    serialConfigMutable()->portConfigs[1].functionMask = FUNCTION_MSP | FUNCTION_BLACKBOX;

    serialBackfillFeatureFields();

    EXPECT_EQ(SERIAL_PORT_USART1, gpsConfig()->gps_uart);
    EXPECT_EQ(SERIAL_PORT_USART3, mspConfig()->msp_uart[0]);
    EXPECT_EQ(SERIAL_PORT_USART3, blackboxConfig()->blackbox_uart);
    // Synthesizer now reproduces the legacy view.
    EXPECT_EQ(FUNCTION_GPS, serialSynthesizeFunctionMask(SERIAL_PORT_USART1));
    EXPECT_EQ((uint32_t)(FUNCTION_MSP | FUNCTION_BLACKBOX),
              serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
}

TEST(SerialFeatureMap, BackfillClearsStaleFeatureFieldsForZeroMaskPort)
{
    resetAllConfigs();
    // Pretend the new PG fields carry a stale claim to UART3 from a
    // previous save, but the legacy mask on UART3 now claims nothing.
    gpsConfigMutable()->gps_uart = SERIAL_PORT_USART3;
    serialConfigMutable()->portConfigs[0].identifier = SERIAL_PORT_USART3;
    serialConfigMutable()->portConfigs[0].functionMask = 0;

    serialBackfillFeatureFields();

    EXPECT_EQ(SERIAL_PORT_NONE, gpsConfig()->gps_uart);
    EXPECT_EQ(0u, serialSynthesizeFunctionMask(SERIAL_PORT_USART3));
}
