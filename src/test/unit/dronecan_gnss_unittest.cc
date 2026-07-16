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
#include <string.h>
#include <math.h>

extern "C" {
    #include "platform.h"

    #include "canard.h"
    #include "io/gps.h"
    #include "io/dronecan/dronecan.h"
    #include "io/dronecan/dronecan_gnss.h"
    #include "io/dronecan/dronecan_msg.h"

    static int64_t stubUnixSeconds;
    static uint16_t stubMillis;
    static bool stubDateTimeCalled;

    void gpsUnixSecondsToDateTime(gpsDateTime_t *dt, int64_t unixSeconds, uint16_t millis)
    {
        stubUnixSeconds = unixSeconds;
        stubMillis = millis;
        stubDateTimeCalled = true;
        memset(dt, 0, sizeof(*dt));
        dt->year = 2021;
        dt->month = 1;
        dt->day = 1;
        dt->millis = millis;
    }

    bool dronecanRegisterSubscriber(const dronecanSubscriber_t *subscriber)
    {
        (void)subscriber;
        return true;
    }

    static timeUs_t mockMicros = 0;

    timeUs_t micros(void) { return mockMicros; }

    // Pulled in as source so the tests can drive the static frame handlers and
    // inspect the seqlock-published cache directly.
    #include "io/dronecan/dronecan_gnss.c"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static void encodeF32(uint8_t *buf, uint32_t bit, float value)
{
    canardEncodeScalar(buf, bit, 32, &value);
}

static void encodeF16(uint8_t *buf, uint32_t bit, float value)
{
    uint16_t raw = canardConvertNativeFloatToFloat16(value);
    canardEncodeScalar(buf, bit, 16, &raw);
}

static void resetCache(void)
{
    memset(&latest, 0, sizeof(latest));
    latestSeq = 0;
    received = false;
    lastUpdateUs = 0;
    auxUpdateUs = 0;
    auxReceived = false;
    mockMicros = 0;
    stubDateTimeCalled = false;
    stubUnixSeconds = 0;
    stubMillis = 0;
}

// Builds a Fix2 payload carrying a 3D fix, a 6-entry (diagonal) covariance and
// pdop, with a UTC gnss_timestamp. Returns the payload length in bytes.
static uint16_t buildFix2(uint8_t *buf)
{
    memset(buf, 0, 64);

    const uint64_t gnssUsec = 1609459200123456ULL; // 2021-01-01T00:00:00.123456Z
    const uint8_t timeStandard = UAVCAN_GNSS_TIME_STANDARD_UTC;
    const int64_t lon_1e8 = 900000000LL;   //   9.0 deg
    const int64_t lat_1e8 = 4500000000LL;  //  45.0 deg
    const int32_t heightMslMm = 100000;    // 100 m
    const uint8_t satsUsed = 12;
    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_3D_FIX;

    canardEncodeScalar(buf, 56, 56, &gnssUsec);
    canardEncodeScalar(buf, 112, 3, &timeStandard);
    canardEncodeScalar(buf, 136, 37, &lon_1e8);
    canardEncodeScalar(buf, 173, 37, &lat_1e8);
    canardEncodeScalar(buf, 237, 27, &heightMslMm);
    encodeF32(buf, 264, 5.0f);   // velN
    encodeF32(buf, 296, 3.0f);   // velE
    encodeF32(buf, 328, -1.0f);  // velD
    canardEncodeScalar(buf, 360, 6, &satsUsed);
    canardEncodeScalar(buf, 366, 2, &status);

    const uint8_t covLength = 6;
    canardEncodeScalar(buf, 378, 6, &covLength);
    encodeF16(buf, 384 + 0 * 16, 4.0f);   // xx  (m^2)
    encodeF16(buf, 384 + 1 * 16, 9.0f);   // yy  (m^2)
    encodeF16(buf, 384 + 2 * 16, 16.0f);  // zz  (m^2)
    encodeF16(buf, 384 + 3 * 16, 0.25f);  // vxx ((m/s)^2)
    encodeF16(buf, 384 + 4 * 16, 0.25f);  // vyy
    encodeF16(buf, 384 + 5 * 16, 1.0f);   // vzz
    encodeF16(buf, 480, 1.5f);            // pdop

    return 62; // 496 bits
}

static uint16_t buildAuxiliary(uint8_t *buf)
{
    memset(buf, 0, 32);
    encodeF16(buf, 32, 0.75f); // hdop
    encodeF16(buf, 48, 1.5f);  // vdop
    return 16;
}

static void feedDefaultFix2(void);

static void feed(void (*handler)(CanardInstance *, CanardRxTransfer *),
                 const uint8_t *payload, uint16_t len)
{
    CanardRxTransfer t;
    memset(&t, 0, sizeof(t));
    t.payload_head = payload;
    t.payload_len = len;
    t.payload_middle = NULL;
    t.payload_tail = NULL;
    handler(NULL, &t);
}

static void feedDefaultFix2(void)
{
    uint8_t buf[64];
    feed(handleFix2, buf, buildFix2(buf));
}

TEST(DronecanGnssTest, Fix2PositionVelocity)
{
    resetCache();

    uint8_t buf[64];
    const uint16_t len = buildFix2(buf);
    feed(handleFix2, buf, len);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ(450000000, sol.llh.lat);
    EXPECT_EQ(90000000, sol.llh.lon);
    EXPECT_EQ(10000, sol.llh.altCm);
    EXPECT_EQ(500, sol.velned.velN);
    EXPECT_EQ(300, sol.velned.velE);
    EXPECT_EQ(-100, sol.velned.velD);
    EXPECT_EQ(583, sol.groundSpeed); // sqrt(34) m/s * 100
    EXPECT_EQ(591, sol.speed3d);     // sqrt(35) m/s * 100
    EXPECT_EQ(309, sol.groundCourse);
    EXPECT_EQ(12, sol.numSat);
}

TEST(DronecanGnssTest, Fix2CovarianceToAccuracy)
{
    resetCache();

    uint8_t buf[64];
    const uint16_t len = buildFix2(buf);
    feed(handleFix2, buf, len);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    // hAcc = sqrt(mean(xx,yy)) = sqrt(6.5) m -> mm
    EXPECT_EQ((uint32_t)(sqrtf(6.5f) * 1000.0f), sol.acc.hAcc);
    // vAcc = sqrt(zz) = 4 m -> mm
    EXPECT_EQ(4000u, sol.acc.vAcc);
    // sAcc = sqrt(mean(vxx,vyy,vzz)) = sqrt(0.5) m/s -> mm/s
    EXPECT_EQ((uint32_t)(sqrtf(0.5f) * 1000.0f), sol.acc.sAcc);
    // pdop 1.5 * 100
    EXPECT_EQ(150, sol.dop.pdop);
}

TEST(DronecanGnssTest, Fix2FullCovarianceMatrix)
{
    resetCache();

    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));

    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_3D_FIX;
    canardEncodeScalar(buf, 366, 2, &status);

    const uint8_t covLength = 36;
    canardEncodeScalar(buf, 378, 6, &covLength);
    // Full 6x6 row-major; only the diagonal is consumed.
    encodeF16(buf, 384 + 0 * 16, 4.0f);
    encodeF16(buf, 384 + 7 * 16, 9.0f);
    encodeF16(buf, 384 + 14 * 16, 16.0f);
    encodeF16(buf, 384 + 21 * 16, 0.25f);
    encodeF16(buf, 384 + 28 * 16, 0.25f);
    encodeF16(buf, 384 + 35 * 16, 1.0f);
    const uint32_t pdopBit = 384 + 36 * 16;
    encodeF16(buf, pdopBit, 1.5f);
    const uint16_t len = (uint16_t)((pdopBit + 16 + 7) / 8);

    feed(handleFix2, buf, len);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ((uint32_t)(sqrtf(6.5f) * 1000.0f), sol.acc.hAcc);
    EXPECT_EQ(4000u, sol.acc.vAcc);
    EXPECT_EQ((uint32_t)(sqrtf(0.5f) * 1000.0f), sol.acc.sAcc);
    EXPECT_EQ(150, sol.dop.pdop);
}

TEST(DronecanGnssTest, Fix2UtcTimestamp)
{
    resetCache();

    uint8_t buf[64];
    const uint16_t len = buildFix2(buf);
    feed(handleFix2, buf, len);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_TRUE(stubDateTimeCalled);
    EXPECT_EQ(1609459200, stubUnixSeconds);
    EXPECT_EQ(123, stubMillis);
    EXPECT_TRUE(sol.dateTime.valid);
}

TEST(DronecanGnssTest, AuxiliaryDop)
{
    resetCache();

    feedDefaultFix2();

    uint8_t aux[32];
    feed(handleAuxiliary, aux, buildAuxiliary(aux));

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ(75, sol.dop.hdop);  // 0.75 * 100
    EXPECT_EQ(150, sol.dop.vdop); // 1.5  * 100
    EXPECT_EQ(150, sol.dop.pdop); // still from Fix2
}

TEST(DronecanGnssTest, Fix2PreservesAuxiliaryDop)
{
    resetCache();

    feedDefaultFix2();

    uint8_t aux[32];
    feed(handleAuxiliary, aux, buildAuxiliary(aux));

    // A fresh Fix2 must not wipe the hdop/vdop published by Auxiliary.
    feedDefaultFix2();

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ(75, sol.dop.hdop);
    EXPECT_EQ(150, sol.dop.vdop);
}

// Anything below a 3D fix is unusable for navigation, so 2D is deliberately
// reported as no fix.
TEST(DronecanGnssTest, Fix2TreatsTwoDFixAsNoFix)
{
    resetCache();

    uint8_t buf[64];
    buildFix2(buf);
    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_2D_FIX;
    canardEncodeScalar(buf, 366, 2, &status);
    feed(handleFix2, buf, 62);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));
    EXPECT_EQ(0, sol.numSat);
}

TEST(DronecanGnssTest, Fix2ScalarCovariance)
{
    resetCache();

    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));

    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_3D_FIX;
    canardEncodeScalar(buf, 366, 2, &status);

    const uint8_t covLength = 1;
    canardEncodeScalar(buf, 378, 6, &covLength);
    encodeF16(buf, 384, 4.0f);
    const uint32_t pdopBit = 384 + 16;
    encodeF16(buf, pdopBit, 1.5f);
    feed(handleFix2, buf, (uint16_t)((pdopBit + 16 + 7) / 8));

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    // A single scalar variance applies to every axis.
    EXPECT_EQ(2000u, sol.acc.hAcc);
    EXPECT_EQ(2000u, sol.acc.vAcc);
    EXPECT_EQ(2000u, sol.acc.sAcc);
    EXPECT_EQ(150, sol.dop.pdop);
}

TEST(DronecanGnssTest, Fix2UpperTriangularCovariance)
{
    resetCache();

    uint8_t buf[128];
    memset(buf, 0, sizeof(buf));

    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_3D_FIX;
    canardEncodeScalar(buf, 366, 2, &status);

    const uint8_t covLength = 21;
    canardEncodeScalar(buf, 378, 6, &covLength);
    // Upper triangle row-major; diagonal entries sit at 0, 6, 11, 15, 18, 20.
    encodeF16(buf, 384 + 0 * 16, 4.0f);
    encodeF16(buf, 384 + 6 * 16, 9.0f);
    encodeF16(buf, 384 + 11 * 16, 16.0f);
    encodeF16(buf, 384 + 15 * 16, 0.25f);
    encodeF16(buf, 384 + 18 * 16, 0.25f);
    encodeF16(buf, 384 + 20 * 16, 1.0f);
    const uint32_t pdopBit = 384 + 21 * 16;
    encodeF16(buf, pdopBit, 1.5f);
    feed(handleFix2, buf, (uint16_t)((pdopBit + 16 + 7) / 8));

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ((uint32_t)(sqrtf(6.5f) * 1000.0f), sol.acc.hAcc);
    EXPECT_EQ(4000u, sol.acc.vAcc);
    EXPECT_EQ((uint32_t)(sqrtf(0.5f) * 1000.0f), sol.acc.sAcc);
    EXPECT_EQ(150, sol.dop.pdop);
}

TEST(DronecanGnssTest, Fix2TruncatedCovarianceIsIgnored)
{
    resetCache();

    uint8_t buf[64];
    memset(buf, 0, sizeof(buf));

    const uint8_t status = UAVCAN_GNSS_FIX2_STATUS_3D_FIX;
    canardEncodeScalar(buf, 366, 2, &status);

    // Claim a full matrix but truncate the payload after six entries: the
    // decoder must not read past the payload, and no accuracy is published.
    const uint8_t covLength = 36;
    canardEncodeScalar(buf, 378, 6, &covLength);
    encodeF16(buf, 384, 4.0f);
    feed(handleFix2, buf, (uint16_t)((384 + 6 * 16) / 8));

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ(0u, sol.acc.hAcc);
    EXPECT_EQ(0u, sol.acc.vAcc);
    EXPECT_EQ(0u, sol.acc.sAcc);
    EXPECT_EQ(0, sol.dop.pdop);
}

TEST(DronecanGnssTest, AuxiliaryDopExpires)
{
    resetCache();

    feedDefaultFix2();

    uint8_t aux[32];
    feed(handleAuxiliary, aux, buildAuxiliary(aux));

    // Fix2 keeps the Auxiliary figures only while they're fresh.
    mockMicros = DRONECAN_GNSS_AUX_FRESH_US + 1;
    feedDefaultFix2();

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_EQ(0, sol.dop.hdop);
    EXPECT_EQ(0, sol.dop.vdop);
    EXPECT_EQ(150, sol.dop.pdop);
}

TEST(DronecanGnssTest, Fix2NonUtcTimestampSkipsDateTime)
{
    resetCache();

    uint8_t buf[64];
    const uint16_t len = buildFix2(buf);
    const uint8_t timeStandard = UAVCAN_GNSS_TIME_STANDARD_GPS;
    canardEncodeScalar(buf, 112, 3, &timeStandard);
    feed(handleFix2, buf, len);

    gpsSolutionData_t sol;
    ASSERT_TRUE(dronecanGnssGetLatest(&sol));

    EXPECT_FALSE(stubDateTimeCalled);
    EXPECT_FALSE(sol.dateTime.valid);
}
