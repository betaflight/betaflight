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

extern "C" {
    #include "platform.h"
    #include "io/radar.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

/* Builds a FormationFlight / INAV MSP2_COMMON_SET_RADAR_POS payload, little-endian */
static void buildPos(uint8_t *p, uint8_t id, uint8_t state, int32_t lat, int32_t lon, int32_t altCm,
                     uint16_t heading, uint16_t speed, uint8_t lq)
{
    p[0] = id;
    p[1] = state;
    memcpy(&p[2], &lat, 4);
    memcpy(&p[6], &lon, 4);
    memcpy(&p[10], &altCm, 4);
    memcpy(&p[14], &heading, 2);
    memcpy(&p[16], &speed, 2);
    p[18] = lq;
}

class RadarTest : public ::testing::Test {
protected:
    void SetUp() override { radarResetForTest(); }
    uint8_t buf[RADAR_POS_PAYLOAD_SIZE];
};

TEST_F(RadarTest, DecodesPayloadFields)
{
    buildPos(buf, 2, RADAR_PEER_STATE_ARMED, 473977420, -85452100, 45012, 271, 1234, 3);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));

    const radarPeer_t *peer = radarFindPeer(2, 1000);
    ASSERT_NE(nullptr, peer);
    EXPECT_EQ(473977420, peer->llh.lat);
    EXPECT_EQ(-85452100, peer->llh.lon);
    EXPECT_EQ(45012, peer->llh.altCm);
    EXPECT_EQ(271, peer->heading);
    EXPECT_EQ(1234, peer->speed);
    EXPECT_EQ(3, peer->lq);
    EXPECT_EQ('B', radarGetPeerLetter(2));
}

TEST_F(RadarTest, RejectsShortPayload)
{
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 1, 1, 0, 0, 0, 4);
    EXPECT_FALSE(radarReceivePos(buf, RADAR_POS_PAYLOAD_SIZE - 1, 1000));
    EXPECT_EQ(nullptr, radarFindPeer(1, 1000));
}

TEST_F(RadarTest, DropsOutOfRangeIdWithoutTouchingOtherSlots)
{
    buildPos(buf, RADAR_MAX_PEERS, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);

    buildPos(buf, RADAR_MAX_PEERS + 1, RADAR_PEER_STATE_ARMED, 99, 99, 0, 0, 0, 4);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));
    buildPos(buf, 0, RADAR_PEER_STATE_ARMED, 99, 99, 0, 0, 0, 4);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));

    const radarPeer_t *peer = radarFindPeer(RADAR_MAX_PEERS, 1000);
    ASSERT_NE(nullptr, peer);
    EXPECT_EQ(10, peer->llh.lat);
    EXPECT_EQ(nullptr, radarFindPeer(0, 1000));
    EXPECT_EQ(nullptr, radarFindPeer(RADAR_MAX_PEERS + 1, 1000));
}

TEST_F(RadarTest, DropsOutOfRangePositionWithoutTouchingTheSlot)
{
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);

    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 900000001, 10, 0, 0, 0, 4);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 10, -1800000001, 0, 0, 0, 4);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, INT32_MIN, INT32_MAX, 0, 0, 0, 4);
    EXPECT_TRUE(radarReceivePos(buf, sizeof(buf), 1000));

    const radarPeer_t *peer = radarFindPeer(1, 1000);
    ASSERT_NE(nullptr, peer);
    EXPECT_EQ(10, peer->llh.lat);
    EXPECT_EQ(10, peer->llh.lon);

    // the limits themselves are valid positions
    buildPos(buf, 2, RADAR_PEER_STATE_ARMED, -900000000, 1800000000, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    peer = radarFindPeer(2, 1000);
    ASSERT_NE(nullptr, peer);
    EXPECT_EQ(-900000000, peer->llh.lat);
    EXPECT_EQ(1800000000, peer->llh.lon);
}

TEST_F(RadarTest, PeerExpiresAfterTimeout)
{
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);

    EXPECT_NE(nullptr, radarFindPeer(1, 1000 + RADAR_PEER_TIMEOUT_MS));
    EXPECT_EQ(nullptr, radarFindPeer(1, 1000 + RADAR_PEER_TIMEOUT_MS + 1));
}

TEST_F(RadarTest, LostStateAndNoPositionAreHidden)
{
    buildPos(buf, 1, RADAR_PEER_STATE_LOST, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    EXPECT_EQ(nullptr, radarFindPeer(1, 1000));

    buildPos(buf, 2, RADAR_PEER_STATE_ARMED, 0, 0, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    EXPECT_EQ(nullptr, radarFindPeer(2, 1000));
}

TEST_F(RadarTest, NextPeerCyclesHealthyPeersOnly)
{
    EXPECT_EQ(0, radarGetNextPeerId(0, 1000));

    buildPos(buf, 2, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    buildPos(buf, 5, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    buildPos(buf, 3, RADAR_PEER_STATE_LOST, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);

    EXPECT_EQ(2, radarGetNextPeerId(0, 1000));
    EXPECT_EQ(5, radarGetNextPeerId(2, 1000));
    EXPECT_EQ(2, radarGetNextPeerId(5, 1000));

    // only one left: stays on it
    buildPos(buf, 5, RADAR_PEER_STATE_LOST, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 1000);
    EXPECT_EQ(2, radarGetNextPeerId(2, 1000));
}

// Own position 47.3977420 N, 8.5455940 E; offsets use 111319.49 m per degree
TEST_F(RadarTest, DistanceScalesLongitudeByOwnLatitude)
{
    const gpsLocation_t own = { .lat = 473977420, .lon = 85455940, .altCm = 0 };
    uint32_t distanceCm;
    int32_t bearing;

    // 150 m east: longitude delta is 150 / (111319.49 * cos(47.3977420 deg)) degrees
    const gpsLocation_t east = { .lat = 473977420, .lon = 85455940 + 19910, .altCm = 0 };
    radarCalcDistanceBearing(&distanceCm, &bearing, &own, &east);
    EXPECT_NEAR(15000, distanceCm, 150);
    EXPECT_NEAR(9000, bearing, 100);

    // 200 m north
    const gpsLocation_t north = { .lat = 473977420 + 17966, .lon = 85455940, .altCm = 0 };
    radarCalcDistanceBearing(&distanceCm, &bearing, &own, &north);
    EXPECT_NEAR(20000, distanceCm, 200);
    EXPECT_TRUE(bearing < 100 || bearing > 35900);

    // 1500 m north-east
    const gpsLocation_t northEast = { .lat = 473977420 + 95281, .lon = 85455940 + 140786, .altCm = 0 };
    radarCalcDistanceBearing(&distanceCm, &bearing, &own, &northEast);
    EXPECT_NEAR(150000, distanceCm, 1500);
    EXPECT_NEAR(4500, bearing, 100);
}

// Own position 17 S, 179.9 E; the peer is 0.2 degrees further east, across the 180 degree meridian
TEST_F(RadarTest, DistanceTakesShortWayAcrossAntimeridian)
{
    const gpsLocation_t own = { .lat = -170000000, .lon = 1799000000, .altCm = 0 };
    const gpsLocation_t peer = { .lat = -170000000, .lon = -1799000000, .altCm = 0 };
    uint32_t distanceCm;
    int32_t bearing;

    // 0.2 * 111319.49 m * cos(17 deg) = 21291 m
    radarCalcDistanceBearing(&distanceCm, &bearing, &own, &peer);
    EXPECT_NEAR(2129100, distanceCm, 21000);
    EXPECT_NEAR(9000, bearing, 100);

    radarCalcDistanceBearing(&distanceCm, &bearing, &peer, &own);
    EXPECT_NEAR(2129100, distanceCm, 21000);
    EXPECT_NEAR(27000, bearing, 100);
}

// +180 and -180 are the same meridian; the raw difference of 360 degrees does not fit in int32_t
TEST_F(RadarTest, DistanceAtOppositeLongitudeLimitsIsZero)
{
    const gpsLocation_t own = { .lat = 0, .lon = 1800000000, .altCm = 0 };
    const gpsLocation_t peer = { .lat = 0, .lon = -1800000000, .altCm = 0 };
    uint32_t distanceCm;
    int32_t bearing;

    radarCalcDistanceBearing(&distanceCm, &bearing, &own, &peer);
    EXPECT_EQ(0u, distanceCm);
    radarCalcDistanceBearing(&distanceCm, &bearing, &peer, &own);
    EXPECT_EQ(0u, distanceCm);
}

// SD canvas 30x16 with INAV defaults: centre (15,8), HUD columns 5..24, rows 3..10
static const radarHudGeometry_t sdGeometry = { 30, 16, 135, 85, 0, 3, 3 };

TEST_F(RadarTest, HudPeerStraightAheadIsAtCentre)
{
    int col, row;
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 0, 200, 0, 0));
    EXPECT_EQ(15, col);
    EXPECT_EQ(8, row);
}

TEST_F(RadarTest, HudPeerToTheRightMovesRight)
{
    int col, row;
    // 15 + 15 * sin(30) / sin(67.5) = 23.1
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 30, 200, 0, 0));
    EXPECT_EQ(23, col);
    EXPECT_EQ(8, row);
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, -30, 200, 0, 0));
    EXPECT_EQ(7, col);
}

TEST_F(RadarTest, HudPeerOutsideViewParksAtEdge)
{
    int col, row;
    // inside the camera FOV but beyond the HUD margin
    EXPECT_FALSE(radarHudProject(&col, &row, &sdGeometry, 60, 200, 0, 0));
    EXPECT_EQ(24, col);
    EXPECT_EQ(7, row);
    // behind, to the left
    EXPECT_FALSE(radarHudProject(&col, &row, &sdGeometry, -100, 200, 0, 0));
    EXPECT_EQ(5, col);
    EXPECT_EQ(7, row);
}

TEST_F(RadarTest, HudPeerAboveIsHigherOnScreen)
{
    int col, row;
    // 10 m above at 200 m: 2.9 deg up, 8 * sin(-2.9) / sin(42.5) = -0.6 -> one row up
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 0, 200, 10, 0));
    EXPECT_EQ(7, row);
    // 45 deg up is clamped to the top HUD row
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 0, 100, 100, 0));
    EXPECT_EQ(3, row);
    // below is lower, clamped to the last HUD row
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 0, 100, -100, 0));
    EXPECT_EQ(10, row);
}

TEST_F(RadarTest, HudPitchAndCameraUptilt)
{
    int col, row;
    // nose down 20 deg: a level peer rises on screen, 8 * sin(-20) / sin(42.5) = -4.05
    EXPECT_TRUE(radarHudProject(&col, &row, &sdGeometry, 0, 200, 0, 20));
    EXPECT_EQ(4, row);
    // a camera tilted up 20 deg cancels that pitch
    radarHudGeometry_t tilted = sdGeometry;
    tilted.cameraUptilt = 20;
    EXPECT_TRUE(radarHudProject(&col, &row, &tilted, 0, 200, 0, 20));
    EXPECT_EQ(8, row);
}

TEST_F(RadarTest, HudScalesToHdCanvas)
{
    const radarHudGeometry_t hd = { 53, 20, 135, 85, 0, 3, 3 };
    int col, row;
    // 26 + 26 * sin(30) / sin(67.5) = 40.1
    EXPECT_TRUE(radarHudProject(&col, &row, &hd, 30, 200, 0, 0));
    EXPECT_EQ(40, col);
    EXPECT_EQ(10, row);
}

TEST_F(RadarTest, WrapsAnglesToPlusMinus180)
{
    EXPECT_EQ(0, radarWrapAngle180(0));
    EXPECT_EQ(180, radarWrapAngle180(180));
    EXPECT_EQ(-170, radarWrapAngle180(190));
    EXPECT_EQ(170, radarWrapAngle180(-190));
    EXPECT_EQ(-90, radarWrapAngle180(270));
    EXPECT_EQ(10, radarWrapAngle180(370));
    EXPECT_EQ(-10, radarWrapAngle180(-370));
}

TEST_F(RadarTest, SurvivesMillisWrap)
{
    buildPos(buf, 1, RADAR_PEER_STATE_ARMED, 10, 10, 0, 0, 0, 4);
    radarReceivePos(buf, sizeof(buf), 0xFFFFFF00);
    EXPECT_NE(nullptr, radarFindPeer(1, 0x00000100));
}
