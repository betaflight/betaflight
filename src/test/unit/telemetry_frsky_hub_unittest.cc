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

extern "C" {
    #include "platform.h"

    #include "common/axis.h"
    #include "common/time.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/sensor.h"
    #include "drivers/serial.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/imu.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/battery.h"
    #include "sensors/sensors.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky_hub.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define FRSKY_HUB_CYCLETIME_US 125000

static int bytesWritten;

static void testWriteByte(const char)
{
    bytesWritten++;
}

// Each hub cycle ends with a tail byte, so a change in bytesWritten means a cycle ran.
static bool cycleRan(timeUs_t timeUs)
{
    const int before = bytesWritten;
    processFrSkyHubTelemetry(timeUs);
    return bytesWritten != before;
}

TEST(TelemetryFrSkyHubTest, CycleTimingAndTimerWraparound)
{
    bytesWritten = 0;
    EXPECT_TRUE(initFrSkyHubTelemetryExternal(testWriteByte));

    timeUs_t now = 1000000;
    EXPECT_TRUE(cycleRan(now));

    // within the cycle time nothing is sent
    EXPECT_FALSE(cycleRan(now + FRSKY_HUB_CYCLETIME_US - 1));

    // normal cadence
    now += FRSKY_HUB_CYCLETIME_US;
    EXPECT_TRUE(cycleRan(now));

    // gap longer than 2^31 us (35.8 min), shorter than 2^32 us: must still start a new cycle
    now += (1u << 31) + 1000000;
    EXPECT_TRUE(cycleRan(now));
    EXPECT_FALSE(cycleRan(now + FRSKY_HUB_CYCLETIME_US - 1));
    EXPECT_TRUE(cycleRan(now + FRSKY_HUB_CYCLETIME_US));

    // gap that crosses the uint32_t wraparound
    now = UINT32_MAX - 1000;
    EXPECT_TRUE(cycleRan(now));
    now += FRSKY_HUB_CYCLETIME_US; // wraps to a small value
    EXPECT_TRUE(cycleRan(now));
}

// STUBS
extern "C" {
    uint8_t armingFlags;
    uint8_t stateFlags;
    attitudeEulerAngles_t attitude;
    acc_t acc;
    baro_t baro;
    gpsSolutionData_t gpsSol;
    float rcCommand[4];
    rxRuntimeState_t rxRuntimeState;
    serialPort_t *telemetrySharedPort;
    const batteryProfile_t *currentBatteryProfile;
    telemetryConfig_t telemetryConfig_System;

    bool sensors(uint32_t) { return false; }
    bool featureIsEnabled(const uint32_t) { return false; }
    timeMs_t millis(void) { return 0; }

    uint8_t calculateBatteryPercentageRemaining(void) { return 0; }
    bool isBatteryVoltageConfigured(void) { return false; }
    uint16_t getBatteryVoltage(void) { return 0; }
    uint16_t getLegacyBatteryVoltage(void) { return 0; }
    uint8_t getBatteryCellCount(void) { return 0; }
    bool isAmperageConfigured(void) { return false; }
    int32_t getAmperage(void) { return 0; }
    int32_t getMAhDrawn(void) { return 0; }
    int32_t getEstimatedAltitudeCm(void) { return 0; }
    throttleStatus_e calculateThrottleStatus(void) { return THROTTLE_LOW; }

    portSharing_e determinePortSharing(serialPortIdentifier_e, serialPortFunction_e) { return PORTSHARING_NOT_SHARED; }
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
    void closeSerialPort(serialPort_t *) {}
    void serialWrite(serialPort_t *, uint8_t) {}

    serialPortIdentifier_e telemetryProviderPort(uint8_t) { return SERIAL_PORT_NONE; }
    bool telemetryCheckRxPortShared(serialPortIdentifier_e, const SerialRXType) { return false; }
    bool telemetryDetermineEnabledState(portSharing_e) { return false; }
    bool telemetryIsSensorEnabled(sensor_e) { return false; }
}
