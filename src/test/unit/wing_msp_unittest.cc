/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

// Golden-vector round-trip tests for MSP2 wing tuning messages. The hex
// arrays here are the cross-repo wire contract -- the same bytes must
// round-trip through the configurator's wing_msp.test.js. When anyone
// reorders a field in either repo, both tests fail in sync.
//
// Canonical values documented in .plan/GOLDEN_VECTOR.md. Negative values
// for signed fields are deliberate (flushes out readU vs readI bugs).

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "common/streambuf.h"
    #include "flight/pid.h"
    #include "msp/msp_wing.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// 39-byte golden vector for MSP2_WING_TUNING.
// See .plan/GOLDEN_VECTOR.md for the pidProfile_t values it encodes.
static const uint8_t WING_TUNING_GOLDEN[] = {
    0x1E, 0x28, 0x00, 0x01,        // S[R], S[P], S[Y], yaw_type
    0x88, 0xFF,                    // angle_pitch_offset (-120)
    0x64,                          // angle_earth_ref
    0x00, 0x01,                    // tpa_mode, tpa_speed_type
    0x20, 0x03, 0x96, 0x00,        // tpa_speed_basic_delay, basic_gravity
    0x60, 0x09,                    // tpa_speed_max_voltage
    0xCE, 0xFF,                    // tpa_speed_pitch_offset (-50)
    0x01, 0x23,                    // tpa_curve_type, stall_throttle
    0x96, 0x00, 0x50, 0x00,        // tpa_curve_pid_thr0, thr100
    0xFB,                          // tpa_curve_expo (-5)
    0x90, 0x01, 0xFA, 0x00, 0x02,  // spa R: center, width, mode
    0x5E, 0x01, 0xC8, 0x00, 0x01,  // spa P
    0x2C, 0x01, 0x96, 0x00, 0x00,  // spa Y
};

// Populate a pidProfile_t with the canonical wing-tuning values.
static void populateCanonicalWingProfile(pidProfile_t *p)
{
    memset(p, 0, sizeof(*p));
    p->pid[PID_ROLL].S = 30;
    p->pid[PID_PITCH].S = 40;
    p->pid[PID_YAW].S = 0;
    p->yaw_type = 1;
    p->angle_pitch_offset = -120;
    p->angle_earth_ref = 100;
    p->tpa_mode = 0;
    p->tpa_speed_type = 1;
    p->tpa_speed_basic_delay = 800;
    p->tpa_speed_basic_gravity = 150;
    p->tpa_speed_max_voltage = 2400;
    p->tpa_speed_pitch_offset = -50;
    p->tpa_curve_type = 1;
    p->tpa_curve_stall_throttle = 35;
    p->tpa_curve_pid_thr0 = 150;
    p->tpa_curve_pid_thr100 = 80;
    p->tpa_curve_expo = -5;
    p->spa_center[FD_ROLL] = 400;
    p->spa_width[FD_ROLL] = 250;
    p->spa_mode[FD_ROLL] = 2;
    p->spa_center[FD_PITCH] = 350;
    p->spa_width[FD_PITCH] = 200;
    p->spa_mode[FD_PITCH] = 1;
    p->spa_center[FD_YAW] = 300;
    p->spa_width[FD_YAW] = 150;
    p->spa_mode[FD_YAW] = 0;
}

TEST(WingMspUnitTest, ConfigEncodesToGoldenVector)
{
    pidProfile_t profile;
    populateCanonicalWingProfile(&profile);

    uint8_t buffer[64] = {0};
    sbuf_t sbuf;
    sbufInit(&sbuf, buffer, buffer + sizeof(buffer));

    serializeWingTuning(&sbuf, &profile);

    const size_t written = (size_t)(sbuf.ptr - buffer);
    EXPECT_EQ(sizeof(WING_TUNING_GOLDEN), written);
    EXPECT_EQ(0, memcmp(buffer, WING_TUNING_GOLDEN, sizeof(WING_TUNING_GOLDEN)));
}

TEST(WingMspUnitTest, ConfigDecodesGoldenVector)
{
    pidProfile_t profile;
    memset(&profile, 0, sizeof(profile));

    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_TUNING_GOLDEN,
             (uint8_t *)WING_TUNING_GOLDEN + sizeof(WING_TUNING_GOLDEN));

    const bool ok = deserializeWingTuning(&sbuf, &profile);
    EXPECT_TRUE(ok);

    pidProfile_t expected;
    populateCanonicalWingProfile(&expected);

    EXPECT_EQ(expected.pid[PID_ROLL].S,          profile.pid[PID_ROLL].S);
    EXPECT_EQ(expected.pid[PID_PITCH].S,         profile.pid[PID_PITCH].S);
    EXPECT_EQ(expected.pid[PID_YAW].S,           profile.pid[PID_YAW].S);
    EXPECT_EQ(expected.yaw_type,                 profile.yaw_type);
    EXPECT_EQ(expected.angle_pitch_offset,       profile.angle_pitch_offset);
    EXPECT_EQ(expected.angle_earth_ref,          profile.angle_earth_ref);
    EXPECT_EQ(expected.tpa_mode,                 profile.tpa_mode);
    EXPECT_EQ(expected.tpa_speed_type,           profile.tpa_speed_type);
    EXPECT_EQ(expected.tpa_speed_basic_delay,    profile.tpa_speed_basic_delay);
    EXPECT_EQ(expected.tpa_speed_basic_gravity,  profile.tpa_speed_basic_gravity);
    EXPECT_EQ(expected.tpa_speed_max_voltage,    profile.tpa_speed_max_voltage);
    EXPECT_EQ(expected.tpa_speed_pitch_offset,   profile.tpa_speed_pitch_offset);
    EXPECT_EQ(expected.tpa_curve_type,           profile.tpa_curve_type);
    EXPECT_EQ(expected.tpa_curve_stall_throttle, profile.tpa_curve_stall_throttle);
    EXPECT_EQ(expected.tpa_curve_pid_thr0,       profile.tpa_curve_pid_thr0);
    EXPECT_EQ(expected.tpa_curve_pid_thr100,     profile.tpa_curve_pid_thr100);
    EXPECT_EQ(expected.tpa_curve_expo,           profile.tpa_curve_expo);
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        EXPECT_EQ(expected.spa_center[i], profile.spa_center[i]);
        EXPECT_EQ(expected.spa_width[i],  profile.spa_width[i]);
        EXPECT_EQ(expected.spa_mode[i],   profile.spa_mode[i]);
    }
}

TEST(WingMspUnitTest, ConfigRoundTrips)
{
    pidProfile_t source;
    populateCanonicalWingProfile(&source);

    uint8_t buffer[64] = {0};
    sbuf_t writer;
    sbufInit(&writer, buffer, buffer + sizeof(buffer));
    serializeWingTuning(&writer, &source);
    const size_t written = (size_t)(writer.ptr - buffer);

    pidProfile_t roundTripped;
    memset(&roundTripped, 0, sizeof(roundTripped));
    sbuf_t reader;
    sbufInit(&reader, buffer, buffer + written);
    EXPECT_TRUE(deserializeWingTuning(&reader, &roundTripped));

    EXPECT_EQ(0, memcmp(&source.pid[PID_ROLL], &roundTripped.pid[PID_ROLL], sizeof(pidf_t)));
    EXPECT_EQ(source.angle_pitch_offset,     roundTripped.angle_pitch_offset);
    EXPECT_EQ(source.tpa_speed_pitch_offset, roundTripped.tpa_speed_pitch_offset);
    EXPECT_EQ(source.tpa_curve_expo,         roundTripped.tpa_curve_expo);
}

TEST(WingMspUnitTest, DeserializeRejectsTruncatedPayload)
{
    pidProfile_t profile;
    memset(&profile, 0, sizeof(profile));

    // Feed only the first 10 bytes of the golden vector.
    sbuf_t sbuf;
    sbufInit(&sbuf, (uint8_t *)WING_TUNING_GOLDEN, (uint8_t *)WING_TUNING_GOLDEN + 10);

    EXPECT_FALSE(deserializeWingTuning(&sbuf, &profile));
}
