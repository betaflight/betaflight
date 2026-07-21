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

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {

    #include "platform.h"

    #include "common/maths.h"
    #include "common/vector.h"

    #include "drivers/osd_symbols.h"

    #include "fc/runtime_config.h"

    #include "flight/imu.h"
    #include "flight/nav_trail.h"

    #include "io/gps.h"

    #include "osd/osd.h"
    #include "osd/osd_elements.h"
    #include "osd/osd_nav_map.h"

    #include "pg/flight_plan.h"
    #include "pg/osd_nav_map.h"

    void navTrailIngestPosition(const vector2_t *posCm);
    void navTrailResetStateForTest(void);

    // test control knobs consumed by the stubs at the bottom of this file
    extern bool testFlightPlanNavActive;
    extern uint8_t testFlightPlanNavIndex;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

class OsdNavMapTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        armingFlags = 0;
        stateFlags = 0;
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        attitude.values.yaw = 0;
        testFlightPlanNavActive = false;
        testFlightPlanNavIndex = 0;
        osdNavMapConfigMutable()->mode = OSD_NAV_MAP_MODE_NORTH_UP;
        osdNavMapConfigMutable()->centre = OSD_NAV_MAP_CENTRE_HOME;
        osdNavMapConfigMutable()->minScaleM = 150;
        memset(flightPlanConfigMutable(), 0, sizeof(flightPlanConfig_t));
        navTrailResetStateForTest();
        osdNavMapResetRenderStateForTest();
    }

    void setCraftEnuM(float eastM, float northM)
    {
        // matches the linear GPS_distance2d stub below
        gpsSol.llh.lat = lrintf(northM * 100.0f / EARTH_ANGLE_TO_CM);
        gpsSol.llh.lon = lrintf(eastM * 100.0f / EARTH_ANGLE_TO_CM);
    }

    void setWaypoint(uint8_t index, waypointType_e type, float eastM, float northM)
    {
        waypoint_t *wp = &flightPlanConfigMutable()->waypoints[index];
        wp->type = type;
        wp->latitude = lrintf(northM * 100.0f / EARTH_ANGLE_TO_CM);
        wp->longitude = lrintf(eastM * 100.0f / EARTH_ANGLE_TO_CM);
        if (flightPlanConfig()->waypointCount < index + 1) {
            flightPlanConfigMutable()->waypointCount = index + 1;
        }
    }
};

//
// trail recorder
//

TEST_F(OsdNavMapTest, TrailRecorderGatesAppendsByDistance)
{
    vector2_t pos = { .x = 0.0f, .y = 0.0f };
    navTrailIngestPosition(&pos);
    EXPECT_EQ(1u, navTrailCount());

    pos.x = 300.0f;    // 3 m: inside the 5 m spacing, not recorded
    navTrailIngestPosition(&pos);
    EXPECT_EQ(1u, navTrailCount());

    pos.x = 600.0f;    // 6 m from the last stored point: recorded
    navTrailIngestPosition(&pos);
    EXPECT_EQ(2u, navTrailCount());
    EXPECT_EQ(6, navTrailPointAt(1)->eastM);
}

TEST_F(OsdNavMapTest, TrailCompactionDoublesSpacingWhenFull)
{
    // fill the ring: every point 5 m apart is exactly at the spacing gate
    for (int i = 0; i < NAV_TRAIL_CAPACITY; i++) {
        const vector2_t pos = { .x = i * 500.0f, .y = 0.0f };
        navTrailIngestPosition(&pos);
    }
    ASSERT_EQ((unsigned)NAV_TRAIL_CAPACITY, navTrailCount());

    // one more point: the trail halves and the spacing doubles to 10 m
    vector2_t pos = { .x = NAV_TRAIL_CAPACITY * 500.0f, .y = 0.0f };
    navTrailIngestPosition(&pos);
    EXPECT_EQ((unsigned)(NAV_TRAIL_CAPACITY / 2 + 1), navTrailCount());

    const unsigned countBefore = navTrailCount();
    pos.x += 500.0f;   // 5 m: below the doubled spacing, ignored
    navTrailIngestPosition(&pos);
    EXPECT_EQ(countBefore, navTrailCount());
    pos.x += 500.0f;   // 10 m from the last stored point: recorded
    navTrailIngestPosition(&pos);
    EXPECT_EQ(countBefore + 1, navTrailCount());
}

TEST_F(OsdNavMapTest, TrailUpdateRecordsWhileArmedAndResetsOnArmEdge)
{
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    timeUs_t timeUs = 1000000;

    // disarmed: nothing is recorded
    setCraftEnuM(0, 0);
    navTrailUpdate(timeUs);
    EXPECT_EQ(0u, navTrailCount());

    armingFlags |= ARMED;
    navTrailUpdate(timeUs += 150000);
    setCraftEnuM(20, 0);
    navTrailUpdate(timeUs += 150000);
    EXPECT_EQ(2u, navTrailCount());

    // the rate limiter swallows a call inside the 100 ms window
    setCraftEnuM(40, 0);
    navTrailUpdate(timeUs += 10000);
    EXPECT_EQ(2u, navTrailCount());

    // disarm, move, re-arm: the old line must not connect to the new launch
    armingFlags = 0;
    navTrailUpdate(timeUs += 150000);
    armingFlags |= ARMED;
    setCraftEnuM(100, 100);
    navTrailUpdate(timeUs += 150000);
    EXPECT_EQ(1u, navTrailCount());
    EXPECT_EQ(100, navTrailPointAt(0)->eastM);
}

TEST_F(OsdNavMapTest, TrailGlitchBeyondRangeDoesNotWipeTheTrail)
{
    // lay down a normal line
    for (int i = 0; i < 5; i++) {
        const vector2_t pos = { .x = i * 600.0f, .y = 0.0f };
        navTrailIngestPosition(&pos);
    }
    ASSERT_EQ(5u, navTrailCount());

    // a wild glitch way past the int16 range, held for many ticks. before the
    // fix this stored a fresh clamped point every call and decimation chewed
    // the real trail away; now it clamps to one point at the edge and the gate
    // sees zero movement after that
    const vector2_t glitch = { .x = 5000000.0f, .y = 5000000.0f };   // 50 km, past the int16 metre range
    for (int i = 0; i < 300; i++) {
        navTrailIngestPosition(&glitch);
    }

    EXPECT_EQ(6u, navTrailCount());
    EXPECT_EQ(INT16_MAX, navTrailPointAt(5)->eastM);
    // the earlier real points survived (stored in whole metres)
    EXPECT_EQ(0, navTrailPointAt(0)->eastM);
    EXPECT_EQ(24, navTrailPointAt(4)->eastM);
}

//
// map scale
//

TEST_F(OsdNavMapTest, ScaleSelectionQuantisesToDrawnSteps)
{
    EXPECT_EQ(20u, osdNavMapSelectScaleM(10));
    EXPECT_EQ(30u, osdNavMapSelectScaleM(21));
    EXPECT_EQ(100u, osdNavMapSelectScaleM(90));
    EXPECT_EQ(400u, osdNavMapSelectScaleM(400));
    EXPECT_EQ(600u, osdNavMapSelectScaleM(401));
    EXPECT_EQ(51200u, osdNavMapSelectScaleM(4000000));
}

//
// renderer
//

class OsdNavMapRenderTest : public OsdNavMapTest {
protected:
    static const int SCREEN_COLS = 30;
    static const int SCREEN_ROWS = 16;
    uint8_t screen[16][30];
    displayPort_t displayPort;
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    void SetUp() override
    {
        OsdNavMapTest::SetUp();
        memset(screen, ' ', sizeof(screen));
        memset(&displayPort, 0, sizeof(displayPort));
        displayPort.cols = SCREEN_COLS;
        displayPort.rows = SCREEN_ROWS;
    }

    // mimic osdDrawSingleElement/osdDisplayActiveElement for one element
    void renderElement(uint8_t posX = 1, uint8_t posY = 1)
    {
        memset(screen, ' ', sizeof(screen));
        osdElementParms_t element;
        int guard = 16;
        do {
            memset(&element, 0, sizeof(element));
            element.item = OSD_NAV_MAP;
            element.elemPosX = posX;
            element.elemPosY = posY;
            element.type = OSD_ELEMENT_TYPE_1;
            element.buff = buff;
            element.osdDisplayPort = &displayPort;
            element.drawElement = true;
            element.rendered = true;
            element.attr = DISPLAYPORT_SEVERITY_NORMAL;
            buff[0] = '\0';

            osdElementNavMap(&element);

            if (element.drawElement) {
                const int x = (uint8_t)(element.elemPosX + element.elemOffsetX);
                const int y = (uint8_t)(element.elemPosY + element.elemOffsetY);
                for (int i = 0; buff[i] && (x + i) < SCREEN_COLS; i++) {
                    if (y >= 0 && y < SCREEN_ROWS && (x + i) >= 0) {
                        screen[y][x + i] = buff[i];
                    }
                }
            }
        } while (!element.rendered && --guard > 0);
        ASSERT_GT(guard, 0) << "element never finished rendering";
    }

    int countGlyph(uint8_t glyph)
    {
        int count = 0;
        for (int y = 0; y < SCREEN_ROWS; y++) {
            for (int x = 0; x < SCREEN_COLS; x++) {
                if (screen[y][x] == glyph) {
                    count++;
                }
            }
        }
        return count;
    }

    bool findGlyph(uint8_t glyph, int *outX, int *outY)
    {
        for (int y = 0; y < SCREEN_ROWS; y++) {
            for (int x = 0; x < SCREEN_COLS; x++) {
                if (screen[y][x] == glyph) {
                    *outX = x;
                    *outY = y;
                    return true;
                }
            }
        }
        return false;
    }
};

TEST_F(OsdNavMapRenderTest, NorthUpProjectionPlacesCraftEastOfHome)
{
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    attitude.values.yaw = 900;   // nose East
    setCraftEnuM(40, 0);         // craft 40 m East of home

    renderElement();

    // border drawn (14x8 compact map)
    EXPECT_EQ(SYM_STICK_OVERLAY_HORIZONTAL, screen[1][4]);
    EXPECT_EQ(SYM_STICK_OVERLAY_HORIZONTAL, screen[8][4]);
    EXPECT_EQ(SYM_STICK_OVERLAY_VERTICAL, screen[4][1]);
    EXPECT_EQ(SYM_STICK_OVERLAY_VERTICAL, screen[4][14]);
    // north-up cue in the top border
    EXPECT_EQ('N', screen[1][2]);

    int homeX, homeY, craftX, craftY;
    ASSERT_TRUE(findGlyph(SYM_HOMEFLAG, &homeX, &homeY));
    ASSERT_TRUE(findGlyph(osdGetDirectionSymbolFromHeading(90), &craftX, &craftY));
    // same row, craft right (East) of home, both inside the border
    EXPECT_EQ(homeY, craftY);
    EXPECT_GT(craftX, homeX);
    EXPECT_GT(craftX, 1);
    EXPECT_LT(craftX, 14);
}

TEST_F(OsdNavMapRenderTest, HeadingUpRotatesTheWorldAroundTheNose)
{
    osdNavMapConfigMutable()->mode = OSD_NAV_MAP_MODE_HEADING_UP;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    setCraftEnuM(0, 60);          // craft 60 m North of home

    // nose North: craft ahead = North = top of map; craft draws as up-arrow
    attitude.values.yaw = 0;
    renderElement();
    int homeX, homeY, craftX, craftY;
    ASSERT_TRUE(findGlyph(SYM_HOMEFLAG, &homeX, &homeY));
    ASSERT_TRUE(findGlyph(osdGetDirectionSymbolFromHeading(0), &craftX, &craftY));
    EXPECT_LT(craftY, homeY);     // craft above home on screen

    // nose South: the world turns; the craft now appears below home
    attitude.values.yaw = 1800;
    renderElement();
    ASSERT_TRUE(findGlyph(SYM_HOMEFLAG, &homeX, &homeY));
    ASSERT_TRUE(findGlyph(osdGetDirectionSymbolFromHeading(0), &craftX, &craftY));
    EXPECT_GT(craftY, homeY);
}

TEST_F(OsdNavMapRenderTest, AutoZoomLatchesOutwardAndAppliesTheFloor)
{
    osdNavMapConfigMutable()->minScaleM = 20;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;

    setCraftEnuM(5, 0);
    renderElement();
    EXPECT_EQ(20u, osdNavMapScaleMForTest());

    // craft flies out to 400 m: the scale expands to keep it on the map
    setCraftEnuM(400, 0);
    renderElement();
    EXPECT_EQ(1200u, osdNavMapScaleMForTest());

    // back near home: anchored expansion holds the wide scale until the
    // content comfortably fits a smaller step...
    setCraftEnuM(40, 0);
    renderElement();
    EXPECT_EQ(1200u, osdNavMapScaleMForTest());

    // ...then zooms back in
    setCraftEnuM(5, 0);
    renderElement();
    EXPECT_EQ(20u, osdNavMapScaleMForTest());

    // the configured floor stops the zoom-in short
    osdNavMapConfigMutable()->minScaleM = 150;
    osdNavMapResetRenderStateForTest();
    renderElement();
    EXPECT_EQ(150u, osdNavMapScaleMForTest());
}

TEST_F(OsdNavMapRenderTest, AutoZoomFitsAllPositionalWaypoints)
{
    osdNavMapConfigMutable()->minScaleM = 20;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    setCraftEnuM(5, 0);

    setWaypoint(0, WAYPOINT_TYPE_FLYOVER, 300, 0);
    renderElement();
    // 300 m East with 1.25 margin needs 750 m of width -> 800 m step
    EXPECT_EQ(800u, osdNavMapScaleMForTest());
}

TEST_F(OsdNavMapRenderTest, AutoZoomIgnoresWaypointsBeyondTheMarkerCap)
{
    osdNavMapConfigMutable()->minScaleM = 20;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    setCraftEnuM(5, 0);

    // waypoint 1 (index 0) is close; a tenth waypoint (index 9) is way out.
    // markers only label 1..9, so the map must not zoom out to chase a marker
    // it will never draw
    setWaypoint(0, WAYPOINT_TYPE_FLYOVER, 60, 0);
    setWaypoint(9, WAYPOINT_TYPE_FLYOVER, 3000, 0);

    renderElement();

    // 60 m fits the 150 floor; not the ~8 km step index 9 would have forced
    EXPECT_EQ(150u, osdNavMapScaleMForTest());
}

TEST_F(OsdNavMapRenderTest, OffMapHomeClampsToEdgeWithDirectionArrow)
{
    osdNavMapConfigMutable()->centre = OSD_NAV_MAP_CENTRE_CRAFT;
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    attitude.values.yaw = 900;
    setCraftEnuM(500, 0);   // craft 500 m East of home: home due West, off-map

    renderElement();

    // the craft-centred map stays at the scale floor; home cannot be drawn
    EXPECT_EQ(150u, osdNavMapScaleMForTest());
    EXPECT_EQ(0, countGlyph(SYM_HOMEFLAG));
    // a West-pointing arrow sits pinned at the left interior edge
    int x, y;
    ASSERT_TRUE(findGlyph(osdGetDirectionSymbolFromHeading(270), &x, &y));
    EXPECT_EQ(2, x);
}

TEST_F(OsdNavMapRenderTest, WaypointMarkersSkipModifiersAndBlinkTheActiveOne)
{
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    setCraftEnuM(5, 5);
    // 400 m floor: the border scale label reads "400M", whose digits cannot
    // be mistaken for the '1'..'3' marker glyphs this test counts
    osdNavMapConfigMutable()->minScaleM = 400;

    setWaypoint(0, WAYPOINT_TYPE_FLYOVER, 80, 0);
    setWaypoint(1, WAYPOINT_TYPE_ALT_CHANGE, 0, 0);   // modifier: lat/lon meaningless
    setWaypoint(2, WAYPOINT_TYPE_FLYBY, -80, 40);

    renderElement();
    EXPECT_EQ(1, countGlyph('1'));
    EXPECT_EQ(0, countGlyph('2'));   // modifier entries draw no marker
    EXPECT_EQ(1, countGlyph('3'));

    // executor flying to plan index 2: that marker blinks while '1' stays
    testFlightPlanNavActive = true;
    testFlightPlanNavIndex = 2;
    int seen3 = 0, missing3 = 0;
    for (int i = 0; i < 8; i++) {
        renderElement();
        EXPECT_EQ(1, countGlyph('1'));
        if (countGlyph('3') > 0) {
            seen3++;
        } else {
            missing3++;
        }
    }
    EXPECT_GT(seen3, 0);
    EXPECT_GT(missing3, 0);
}

TEST_F(OsdNavMapRenderTest, TrailSwitchbackKeepsBothCrossingsOfAColumn)
{
    stateFlags |= GPS_FIX | GPS_FIX_HOME;
    osdNavMapConfigMutable()->minScaleM = 150;

    // out-and-back: east along the north edge, down the far side, west along
    // the south edge. the two shallow legs cross the same columns at rows well
    // apart - the column dedup has to keep both, not blank out the return leg
    const float pts[][2] = {
        { -40.0f,  40.0f }, {  40.0f,  40.0f },   // north edge, west -> east
        {  40.0f, -40.0f },                        // down the east side (steep)
        { -40.0f, -40.0f },                        // south edge, east -> west
    };
    for (auto &p : pts) {
        const vector2_t v = { .x = p[0] * 100.0f, .y = p[1] * 100.0f };
        navTrailIngestPosition(&v);
    }
    setCraftEnuM(-40, -40);   // craft on the last point so there's no stray tail

    renderElement();

    // some column has to carry an AH-bar stroke in two different rows; before
    // the fix the return leg was suppressed and no column ever did
    int columnsWithTwoRows = 0;
    for (int x = 0; x < SCREEN_COLS; x++) {
        int rowsWithBar = 0;
        for (int y = 0; y < SCREEN_ROWS; y++) {
            const uint8_t g = screen[y][x];
            if (g >= SYM_AH_BAR9_0 && g <= (uint8_t)(SYM_AH_BAR9_0 + 8)) {
                rowsWithBar++;
            }
        }
        if (rowsWithBar >= 2) {
            columnsWithTwoRows++;
        }
    }
    EXPECT_GT(columnsWithTwoRows, 0);
}

// STUBS

extern "C" {
    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;

    bool testFlightPlanNavActive = false;
    uint8_t testFlightPlanNavIndex = 0;

    bool flightPlanNavIsActive(void) { return testFlightPlanNavActive; }
    uint8_t flightPlanNavGetCurrentIndex(void) { return testFlightPlanNavIndex; }

    // matches the firmware conversion: 1e-7 degree steps to cm
    void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
    {
        distance->x = (to->lon - from->lon) * EARTH_ANGLE_TO_CM;
        distance->y = (to->lat - from->lat) * EARTH_ANGLE_TO_CM;
    }

    // renderer support stubs (the real ones live in osd_elements.c)
    uint8_t osdGetDirectionSymbolFromHeading(int heading)
    {
        heading = ((heading % 360) + 360) % 360;
        int direction = (heading * 16 + 180) / 360;
        direction %= 16;
        direction = (16 - direction + 8) % 16;
        return SYM_ARROW_SOUTH + direction;
    }

    float osdGetMetersToSelectedUnit(int32_t meters) { return meters; }
    char osdGetMetersToSelectedUnitSymbol(void) { return 'M'; }
}
