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

// serial_feature_map.c built the way a cloud build without the VTX option
// builds it: OSD and MSP DisplayPort, but no USE_VTX_* at all.  A legacy
// `serial` line carrying FUNCTION_VTX_MSP (DJI / HD goggles) must still leave
// the MSP DisplayPort with a UART to draw over.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "common/utils.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/msp.h"
    #include "pg/gps.h"
    #include "pg/rx.h"

    #include "blackbox/blackbox.h"

    #include "osd/osd.h"

    #include "io/serial.h"
    #include "io/serial_feature_map.h"

    #include "telemetry/telemetry.h"

    PG_REGISTER(mspConfig_t, mspConfig, PG_MSP_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
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

    bool serialPortFunctionsConflict(serialPortIdentifier_e identifier)
    {
        UNUSED(identifier);
        return false;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

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

    memset(telemetryConfigMutable(), 0, sizeof(*telemetryConfigMutable()));
    for (unsigned i = 0; i < MAX_TELEMETRY_PROVIDERS; i++) {
        telemetryConfigMutable()->providers[i].protocol = TELEMETRY_PROTOCOL_NONE;
        telemetryConfigMutable()->providers[i].uart = SERIAL_PORT_NONE;
    }

    memset(osdConfigMutable(), 0, sizeof(*osdConfigMutable()));
    osdConfigMutable()->osd_uart = SERIAL_PORT_NONE;
    osdConfigMutable()->osd_custom_text_uart = SERIAL_PORT_NONE;
    osdConfigMutable()->osd_custom_text_baud = BAUD_AUTO;

    memset(serialConfigMutable(), 0, sizeof(*serialConfigMutable()));
}

} // namespace

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineGivesTheOsdItsPort)
{
    resetAllConfigs();

    // serial UART2 131073 ...  (MSP + VTX_MSP), the usual DJI O3/O4 line.
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP | FUNCTION_VTX_MSP));

    EXPECT_EQ(SERIAL_PORT_USART2, mspConfig()->msp_uart[0]);
    EXPECT_EQ(SERIAL_PORT_USART2, osdConfig()->osd_uart);
}

TEST(SerialFeatureMapNoVtx, PastedDiffKeepsTheOsdPortThroughLaterSettings)
{
    resetAllConfigs();

    // A 2026.6 diff in paste order: the serial lines first, the OSD device later.
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_RX_SERIAL));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP | FUNCTION_VTX_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_UART5, FUNCTION_GPS));
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MSP;

    EXPECT_EQ(SERIAL_PORT_USART2, osdConfig()->osd_uart);
    EXPECT_EQ(SERIAL_PORT_USART1, rxConfig()->rx_uart);
    EXPECT_EQ(SERIAL_PORT_UART5, gpsConfig()->gps_uart);
    EXPECT_TRUE(serialSynthesizeFunctionMask(SERIAL_PORT_USART2) & FUNCTION_MSP);
}

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineLeavesAnAssignedOsdPortAlone)
{
    resetAllConfigs();
    osdConfigMutable()->osd_uart = SERIAL_PORT_UART4;

    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP | FUNCTION_VTX_MSP));

    EXPECT_EQ(SERIAL_PORT_UART4, osdConfig()->osd_uart);
}

TEST(SerialFeatureMapNoVtx, PlainMspLineDoesNotClaimTheOsd)
{
    resetAllConfigs();

    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP));

    EXPECT_EQ(SERIAL_PORT_NONE, osdConfig()->osd_uart);
}

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineFillsTheLastMspSlot)
{
    resetAllConfigs();

    // USB, Bluetooth and goggles: three MSP ports, three slots.
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP | FUNCTION_VTX_MSP));

    EXPECT_EQ(SERIAL_PORT_USB_VCP, mspConfig()->msp_uart[0]);
    EXPECT_EQ(SERIAL_PORT_USART1, mspConfig()->msp_uart[1]);
    EXPECT_EQ(SERIAL_PORT_USART3, mspConfig()->msp_uart[2]);
    EXPECT_EQ(SERIAL_PORT_USART3, osdConfig()->osd_uart);
}

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineAppliesWhenEveryMspSlotIsTaken)
{
    resetAllConfigs();

    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP));

    // A diff sets osd_displayport_device after its serial lines, so the OSD
    // is still on a non-MSP device when the goggles line arrives.
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_AUTO;

    // No slot left: osd_uart takes the port and the conversion selects the
    // MSP DisplayPort, so the port is still opened as MSP.
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP | FUNCTION_VTX_MSP));

    EXPECT_EQ(SERIAL_PORT_USART3, osdConfig()->osd_uart);
    EXPECT_EQ(OSD_DISPLAYPORT_DEVICE_MSP, osdConfig()->displayPortDevice);
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        EXPECT_NE(SERIAL_PORT_USART3, mspConfig()->msp_uart[i]);
    }

    serialPortIdentifier_e implied[4];
    const unsigned count = serialImpliedMspPorts(implied, ARRAYLEN(implied));
    ASSERT_EQ(1u, count);
    EXPECT_EQ(SERIAL_PORT_USART3, implied[0]);
}

TEST(SerialFeatureMapNoVtx, PlainMspLineIsStillRejectedWhenEveryMspSlotIsTaken)
{
    resetAllConfigs();

    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP));

    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP));
}

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineIsRejectedWhenTheOsdHasAnotherPortAndNoSlotIsFree)
{
    resetAllConfigs();
    osdConfigMutable()->osd_uart = SERIAL_PORT_UART4;

    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USB_VCP, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART1, FUNCTION_MSP));
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART2, FUNCTION_MSP));

    EXPECT_FALSE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP | FUNCTION_VTX_MSP));
    EXPECT_EQ(SERIAL_PORT_UART4, osdConfig()->osd_uart);
}

TEST(SerialFeatureMapNoVtx, LegacyMspVtxLineWithAFreeSlotLeavesTheOsdDeviceAlone)
{
    resetAllConfigs();
    osdConfigMutable()->displayPortDevice = OSD_DISPLAYPORT_DEVICE_MAX7456;

    // A slot opens the port as MSP already; the OSD device stays the user's.
    ASSERT_TRUE(serialApplyFunctionMask(SERIAL_PORT_USART3, FUNCTION_MSP | FUNCTION_VTX_MSP));

    EXPECT_EQ(SERIAL_PORT_USART3, mspConfig()->msp_uart[0]);
    EXPECT_EQ(OSD_DISPLAYPORT_DEVICE_MAX7456, osdConfig()->displayPortDevice);
}
