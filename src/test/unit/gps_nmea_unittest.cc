/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>


#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"
    #include "config/feature.h"
    #include "drivers/time.h"
    #include "fc/runtime_config.h"
    #include "io/beeper.h"
    #include "io/dashboard.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "scheduler/scheduler.h"

    bool gpsNewFrameNMEA(char c);

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    const uint32_t baudRates[BAUD_COUNT] = { 0 };

    timeUs_t micros(void) { return 0; }
    timeMs_t millis(void) { return 0; }
    uint32_t clockMicrosToCycles(uint32_t) { return 0; }
    uint32_t getCycleCounter(void) { return 0; }
    void beeper(beeperMode_e) {}
    void beeperConfirmationBeeps(uint8_t) {}
    void dashboardShowFixedPage(pageId_e) {}
    void dashboardUpdate(timeUs_t) {}
    bool featureIsEnabled(const uint32_t) { return true; }
    void rescheduleTask(taskId_e, timeDelta_t) {}
    void schedulerSetNextStateTime(timeDelta_t) {}

    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
    baudRate_e lookupBaudRateIndex(uint32_t) { return BAUD_AUTO; }
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr,
        void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
    uint32_t serialGetBaudRate(serialPort_t *) { return 0; }
    void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
    void serialPrint(serialPort_t *, const char *) {}
    uint8_t serialRead(serialPort_t *) { return 0; }
    uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
    void serialSetBaudRate(serialPort_t *, uint32_t) {}
    void serialSetMode(serialPort_t *, portMode_e) {}
    serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_INVALID; }
    void serialWrite(serialPort_t *, uint8_t) {}
    void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
    void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class GpsNmeaTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        stateFlags = 0;
        memset(&gpsSol, 0, sizeof(gpsSol));
    }

    bool feed(const char *body)
    {
        uint8_t checksum = 0;
        for (const char *p = body; *p; p++) {
            checksum ^= *p;
        }
        char sentence[128];
        snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", body, checksum);
        bool nav = false;
        for (const char *p = sentence; *p; p++) {
            nav |= gpsNewFrameNMEA(*p);
        }
        return nav;
    }
};

static const char *GGA_FIX = "GNGGA,123519.00,4807.03800,N,01131.00000,E,1,08,0.9,545.4,M,46.9,M,,";
static const char *GGA_NO_FIX = "GNGGA,123519.00,,,,,0,00,99.99,,,,,,";
static const char *GSA_2D = "GNGSA,A,2,04,05,09,,,,,,,,,,2.5,1.3,2.1";
static const char *GSA_3D = "GNGSA,A,3,04,05,09,12,24,,,,,,,,2.5,1.3,2.1";
static const char *GSA_NONE = "GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99";

TEST_F(GpsNmeaTest, GgaFixWithoutGsaIsAFix)
{
    EXPECT_TRUE(feed(GGA_FIX));
    EXPECT_TRUE(STATE(GPS_FIX));
    EXPECT_EQ(gpsSol.numSat, 8);
    EXPECT_EQ(gpsSol.llh.altCm, 54540);
}

TEST_F(GpsNmeaTest, GgaNoFixIsNoFix)
{
    feed(GGA_NO_FIX);
    EXPECT_FALSE(STATE(GPS_FIX));
}

TEST_F(GpsNmeaTest, Gsa3dKeepsTheFix)
{
    feed(GSA_3D);
    feed(GGA_FIX);
    EXPECT_TRUE(STATE(GPS_FIX));
}

TEST_F(GpsNmeaTest, Gsa2dIsNoFix)
{
    feed(GSA_2D);
    feed(GGA_FIX);
    EXPECT_FALSE(STATE(GPS_FIX));

    feed(GSA_3D);
    feed(GGA_FIX);
    EXPECT_TRUE(STATE(GPS_FIX));
}

TEST_F(GpsNmeaTest, GsaNoFixIsNoFix)
{
    feed(GSA_NONE);
    feed(GGA_FIX);
    EXPECT_FALSE(STATE(GPS_FIX));
}

TEST_F(GpsNmeaTest, BadChecksumDoesNotChangeTheFix)
{
    feed(GSA_3D);
    feed(GGA_FIX);
    ASSERT_TRUE(STATE(GPS_FIX));

    const char *corrupt = "$GNGGA,123519.00,,,,,0,00,99.99,,,,,,*00\r\n";
    for (const char *p = corrupt; *p; p++) {
        gpsNewFrameNMEA(*p);
    }
    EXPECT_TRUE(STATE(GPS_FIX));
}
