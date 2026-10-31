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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "build/build_config.h"
    #include "build/debug.h"
    #include "common/time.h"
    #include "drivers/serial.h"
    #include "fc/runtime_config.h"
    #include "io/beeper.h"
    #include "io/dashboard.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/gps.h"
    #include "pg/gps_rescue.h"
    #include "scheduler/scheduler.h"

    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);

    uint32_t grab_fields(const char *src, uint8_t mult);
    bool gpsNewFrameNMEA(char c);

    // The RMC handler seeds the RTC once; capture what it would set.
    static dateTime_t capturedDateTime;
    static bool rtcSetCalled;
    bool rtcHasTime(void) { return false; }
    bool rtcSetDateTime(dateTime_t *dt)
    {
        capturedDateTime = *dt;
        rtcSetCalled = true;
        return true;
    }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

struct grabFieldsCase_t {
    const char *field;
    uint8_t mult;
    uint32_t expected;
};

TEST(GpsNmeaTest, GrabFieldsScalesToTheRequestedDecimals)
{
    static const grabFieldsCase_t cases[] = {
        // exactly the assumed number of decimals
        { "123.4",     1, 1234 },
        { "22.4",      1, 224 },
        { "123519.00", 2, 12351900 },
        { "1.50",      2, 150 },
        // more decimals than asked for: truncated, not rounded
        { "123.45",    1, 1234 },
        { "0.004",     1, 0 },
        { "123519.123", 2, 12351912 },
        // fewer decimals than asked for: padded
        { "123",       1, 1230 },
        { "22",        1, 220 },
        { "123519",    2, 12351900 },
        { "123519.5",  2, 12351950 },
        { "1.5",       2, 150 },
        { "1",         2, 100 },
        // integer fields
        { "12",        0, 12 },
        { "12.9",      0, 12 },
        { "",          0, 0 },
        { "",          2, 0 },
        // sign
        { "-12.3",     1, (uint32_t)-123 },
        { "-5",        1, (uint32_t)-50 },
    };

    for (const grabFieldsCase_t &c : cases) {
        EXPECT_EQ(c.expected, grab_fields(c.field, c.mult)) << "field \"" << c.field << "\" mult " << (int)c.mult;
    }
}

TEST(GpsNmeaTest, GrabFieldsHandlesFieldsThatFillTheBuffer)
{
    // 15 characters is the most gpsNewFrameNMEA() stores in a field
    EXPECT_EQ(123456789, grab_fields("1234567.8901234", 2)) << "decimals truncated";
    EXPECT_EQ(50, grab_fields("0000000000000.5", 2)) << "decimal point near the end";
    EXPECT_EQ(1234567890, grab_fields("1234567890.0000", 0)) << "integer part only";
}

TEST(GpsNmeaTest, GrabFieldsGivesZeroWhenTheValueDoesNotFit)
{
    EXPECT_EQ(4294967295u, grab_fields("4294967295", 0)) << "largest value that fits";
    EXPECT_EQ(0, grab_fields("4294967296", 0)) << "one more does not";
    EXPECT_EQ(0, grab_fields("999999999999999", 1)) << "15 digits";
    EXPECT_EQ(0, grab_fields("4294967295", 1)) << "padding would overflow";
    EXPECT_EQ(0, grab_fields("429496729.6", 2)) << "padding the fraction would overflow";
}

// Feed one complete sentence, appending the checksum NMEA requires.
static bool feedSentence(const char *body)
{
    uint8_t parity = 0;
    for (const char *p = body + 1; *p; p++) { // skip the leading '$'
        parity ^= *p;
    }
    char sentence[128];
    snprintf(sentence, sizeof(sentence), "%s*%02X\r\n", body, parity);

    bool navMessage = false;
    for (const char *p = sentence; *p; p++) {
        navMessage |= gpsNewFrameNMEA(*p);
    }
    return navMessage;
}

TEST(GpsNmeaTest, GgaAltitudeIsScaledWhateverTheDecimalCount)
{
    memset(&gpsSol, 0, sizeof(gpsSol));

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    EXPECT_EQ(54540, gpsSol.llh.altCm) << "one decimal";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545,M,46.9,M,,");
    EXPECT_EQ(54500, gpsSol.llh.altCm) << "no decimals";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.432,M,46.9,M,,");
    EXPECT_EQ(54540, gpsSol.llh.altCm) << "three decimals";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-12.3,M,46.9,M,,");
    EXPECT_EQ(-1230, gpsSol.llh.altCm) << "below sea level";
}

TEST(GpsNmeaTest, GgaAltitudeThatDoesNotFitInCentimetresIsNotPublished)
{
    memset(&gpsSol, 0, sizeof(gpsSol));

    // 21474836.4 m is the largest altitude whose centimetres fit int32_t
    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,21474836.4,M,46.9,M,,");
    EXPECT_EQ(2147483640, gpsSol.llh.altCm) << "largest that fits";

    // 42949673 m fits grab_fields() as 429496730 dm, but times ten wraps to 4 cm
    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,42949673,M,46.9,M,,");
    EXPECT_EQ(0, gpsSol.llh.altCm) << "centimetres would wrap";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,999999999,M,46.9,M,,");
    EXPECT_EQ(0, gpsSol.llh.altCm) << "far out of range";

    // 429496729.5 m is 4294967295 dm, which a signed cast would read as -1 dm
    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,429496729.5,M,46.9,M,,");
    EXPECT_EQ(0, gpsSol.llh.altCm) << "top of the unsigned range is not a small negative";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-429496729.5,M,46.9,M,,");
    EXPECT_EQ(0, gpsSol.llh.altCm) << "same magnitude, negative";

    feedSentence("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-21474836.4,M,46.9,M,,");
    EXPECT_EQ(-2147483640, gpsSol.llh.altCm) << "most negative that fits";
}

TEST(GpsNmeaTest, RmcSpeedIsScaledWhateverTheDecimalCount)
{
    memset(&gpsSol, 0, sizeof(gpsSol));

    feedSentence("$GPRMC,123519.00,A,4807.038,N,01131.000,E,22.4,084.4,230394,003.1,W");
    EXPECT_EQ(1152, gpsSol.groundSpeed) << "one decimal: 22.4 kn";

    feedSentence("$GPRMC,123519.00,A,4807.038,N,01131.000,E,22,084.4,230394,003.1,W");
    EXPECT_EQ(1131, gpsSol.groundSpeed) << "no decimals: 22 kn";
}

TEST(GpsNmeaTest, RmcTimeIsScaledWhateverTheDecimalCount)
{
    struct {
        const char *name;
        const char *time;
        uint16_t millis;
    } cases[] = {
        { "two decimals",  "123519.00", 0 },
        { "one decimal",   "123519.5",  500 },
        { "no fraction",   "123519",    0 },
        { "three decimals","123519.123", 120 },
    };

    for (const auto &c : cases) {
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&capturedDateTime, 0, sizeof(capturedDateTime));
        rtcSetCalled = false;

        char sentence[96];
        snprintf(sentence, sizeof(sentence), "$GPRMC,%s,A,4807.038,N,01131.000,E,22.4,084.4,010125,003.1,W", c.time);
        feedSentence(sentence);

        ASSERT_TRUE(rtcSetCalled) << c.name;
        EXPECT_EQ(2025, capturedDateTime.year) << c.name;
        EXPECT_EQ(1, capturedDateTime.month) << c.name;
        EXPECT_EQ(1, capturedDateTime.day) << c.name;
        EXPECT_EQ(12, capturedDateTime.hours) << c.name;
        EXPECT_EQ(35, capturedDateTime.minutes) << c.name;
        EXPECT_EQ(19, capturedDateTime.seconds) << c.name;
        EXPECT_EQ(c.millis, capturedDateTime.millis) << c.name;
    }
}

// STUBS
extern "C" {
uint8_t stateFlags = 0;
uint8_t armingFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
const uint32_t baudRates[] = { 0, 9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000 };

uint32_t millis(void) { return 0; }
uint32_t micros(void) { return 0; }
uint32_t getCycleCounter(void) { return 0; }
uint32_t clockMicrosToCycles(uint32_t micros) { return micros; }
bool featureIsEnabled(uint32_t) { return false; }
bool sensors(uint32_t) { return false; }
void sensorsSet(uint32_t) {}
void sensorsClear(uint32_t) {}
void beeper(beeperMode_e) {}
void dashboardUpdate(timeUs_t) {}
void dashboardShowFixedPage(pageId_e) {}
void schedulerSetNextStateTime(timeDelta_t) {}
void rescheduleTask(taskId_e, timeDelta_t) {}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) { return NULL; }
serialType_e serialType(serialPortIdentifier_e) { return SERIALTYPE_UART; }
baudRate_e lookupBaudRateIndex(uint32_t) { return BAUD_9600; }
uint32_t serialGetBaudRate(serialPort_t *) { return 9600; }
void serialSetBaudRate(serialPort_t *, uint32_t) {}
void serialSetMode(serialPort_t *, portMode_e) {}
void serialWrite(serialPort_t *, uint8_t) {}
void serialWriteBuf(serialPort_t *, const uint8_t *, int) {}
void serialPrint(serialPort_t *, const char *) {}
uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
uint8_t serialRead(serialPort_t *) { return 0; }
bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }
void waitForSerialPortToFinishTransmitting(serialPort_t *) {}
void serialPassthrough(serialPort_t *, serialPort_t *, serialConsumer *, serialConsumer *) {}
}
