/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"

    #include "common/maths.h"
    #include "common/printf.h"
    #include "common/vector.h"

    #include "drivers/osd_symbols.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/nav_hud.h"
#include "flight/nav_mission.h"
#include "pg/pos_hold.h"
#include "pg/gps_rescue.h"

    #include "io/gps.h"

    #include "osd/osd.h"
    #include "osd/osd_elements.h"
    #include "osd/osd_nav_hud.h"

    #include "pg/nav_hud.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    PG_REGISTER(navHudConfig_t, navHudConfig, PG_NAV_HUD_CONFIG, 0);
    PG_REGISTER(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    void navHudIngestFix(const vector2_t *posCm, float intervalS, bool rescueFlying, timeUs_t nowUs);

    // test control knobs consumed by the stubs at the bottom of this file
    extern uint32_t testMicros;
    extern rescuePhase_e testRescuePhase;
    extern rescueFailureState_e testRescueFailure;
    extern float testTargetAltitudeCm;
    extern float testTargetVelocityCmS;
    extern float testDescentDistanceCm;
    extern bool testRescueIsOK;
    extern bool canUseGPSHeading;
    extern bool testMagCalibrated;
    extern float testAltitudeCm;
    extern bool testFailsafeActive;
    extern float testStickDeflection;
    extern gpsLocation_t testMissionTarget;
    extern float testMissionSanityCm;
    extern int testMissionTargetSets;
    extern bool testMissionSwitch;
    void navMissionResetForTest(void);
    float navMissionCarrotSpeedForTest(void);
    extern float testVerticalVelCmS;
    extern bool testTargetPosValid;
    extern vector2_t testTargetPos;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static void configureDefaults(void)
{
    navHudConfigMutable()->mode = NAV_HUD_MODE_STANDARD;
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->center = NAV_HUD_CENTER_AUTO;
    navHudConfigMutable()->autoZoom = 1;
    navHudConfigMutable()->rangeRing = 1;
    navHudConfigMutable()->fixedScaleM = 400;
    extern bool testWaypointSwitch;
    testWaypointSwitch = false;
    navMissionResetForTest();
    posHoldConfigMutable()->deadband = 5;
    testFailsafeActive = false;
    testStickDeflection = 0.0f;
    testMissionTargetSets = 0;
    navHudConfigMutable()->breadcrumbs = 1;
    navHudConfigMutable()->breadcrumbCount = 240;
    navHudConfigMutable()->breadcrumbSpacingM = 10;
    navHudConfigMutable()->projectedTrack = 0;
    navHudConfigMutable()->homeLine = 1;
    navHudConfigMutable()->rescueRoute = 1;
    navHudConfigMutable()->rescueExpand = 0;
    navHudConfigMutable()->showEta = 1;
    navHudConfigMutable()->showXtrack = 1;
    navHudConfigMutable()->showTargets = 1;
    navHudConfigMutable()->showGpsHealth = 1;
    navHudConfigMutable()->showSpeed = 1;
}

class NavHudTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        configureDefaults();
        armingFlags = 0;
        stateFlags = 0;
        flightModeFlags = 0;
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        attitude.values.yaw = 0;
        testMicros = 1000000;
        testRescuePhase = RESCUE_IDLE;
        testRescueFailure = RESCUE_HEALTHY;
        testRescueIsOK = true;
        canUseGPSHeading = true;
        testMagCalibrated = false;
        testAltitudeCm = 0;
        testVerticalVelCmS = 0;
        testTargetPosValid = false;
        navHudInit();
        osdNavHudResetRenderStateForTest();
    }

    void arm()
    {
        armingFlags |= ARMED;
        stateFlags |= GPS_FIX | GPS_FIX_HOME;
        gpsSol.numSat = 18;
        gpsSol.dop.hdop = 90;
        gpsSol.navIntervalMs = 100;
        update(); // latch the home reference, as navHudOnGpsNewData() would
    }

    void setCraftLL(float eastM, float northM)
    {
        gpsSol.llh.lat = lrintf(northM * 100.0f / EARTH_ANGLE_TO_CM);
        gpsSol.llh.lon = lrintf(eastM * 100.0f / EARTH_ANGLE_TO_CM);
    }

    void feedFix(float eastM, float northM, float intervalS = 0.1f, bool rescue = false)
    {
        const vector2_t pos = { .x = eastM * 100.0f, .y = northM * 100.0f };
        testMicros += (uint32_t)(intervalS * 1e6f);
        navHudIngestFix(&pos, intervalS, rescue, testMicros);
    }

    const navHudState_t *update()
    {
        testMicros += 60000; // beyond the internal 50 ms rate limit
        navHudUpdate(testMicros);
        return navHudGetState();
    }
};

//
// pure helper math
//

TEST_F(NavHudTest, WrapHelpers)
{
    EXPECT_EQ(-170, navHudWrapDeg180(190));
    EXPECT_EQ(170, navHudWrapDeg180(-190));
    EXPECT_EQ(0, navHudWrapDeg180(360));
    EXPECT_EQ(-180, navHudWrapDeg180(180));
    EXPECT_EQ(350, navHudWrapDeg360(-10));
    EXPECT_EQ(0, navHudWrapDeg360(720));
    EXPECT_EQ(359, navHudWrapDeg360(-361));
}

TEST_F(NavHudTest, BearingBetweenPoints)
{
    const vector2_t origin = { .x = 0, .y = 0 };
    vector2_t target = { .x = 100, .y = 100 };
    EXPECT_EQ(45, navHudBearingDeg(&origin, &target));
    target = { .x = -100, .y = 0 };
    EXPECT_EQ(270, navHudBearingDeg(&origin, &target));
    target = { .x = 0, .y = -50 };
    EXPECT_EQ(180, navHudBearingDeg(&origin, &target));
}

TEST_F(NavHudTest, CrossTrackSignAndMagnitude)
{
    // rescue starts 1000 cm North of home; track home runs due South.
    const vector2_t start = { .x = 0, .y = 1000 };
    vector2_t craft = { .x = 100, .y = 500 };
    // heading South, East is to the left of track
    EXPECT_EQ(-100, navHudCrossTrackCm(&start, &craft));
    craft = { .x = -100, .y = 500 };
    EXPECT_EQ(100, navHudCrossTrackCm(&start, &craft));
    craft = { .x = 0, .y = 500 };
    EXPECT_EQ(0, navHudCrossTrackCm(&start, &craft));
}

TEST_F(NavHudTest, AutoScaleQuantisation)
{
    EXPECT_EQ(20u, navHudSelectAutoScaleM(10));
    EXPECT_EQ(30u, navHudSelectAutoScaleM(21));
    EXPECT_EQ(100u, navHudSelectAutoScaleM(90));
    EXPECT_EQ(400u, navHudSelectAutoScaleM(400));
    EXPECT_EQ(600u, navHudSelectAutoScaleM(401));
    EXPECT_EQ(51200u, navHudSelectAutoScaleM(4000000));
}

//
// breadcrumb trail
//

TEST_F(NavHudTest, BreadcrumbSpacingIsRespected)
{
    arm();
    feedFix(0, 0);
    feedFix(3, 0);    // < 10 m from previous point: not recorded
    feedFix(6, 0);
    EXPECT_EQ(1u, navHudTrailCount());
    feedFix(12, 0);   // >= 10 m: recorded
    EXPECT_EQ(2u, navHudTrailCount());
    EXPECT_EQ(12, navHudTrailPointAt(1)->eastM);
}

TEST_F(NavHudTest, BreadcrumbCompactionDoublesSpacing)
{
    navHudConfigMutable()->breadcrumbCount = 8;
    arm();
    for (int i = 0; i <= 9; i++) {
        feedFix(i * 10.0f, 0);
    }
    // the 9th push hit the 8 point cap and forced a compaction to 4 points,
    // then 80 was pushed; 90 was dropped by the now-doubled 20 m spacing
    ASSERT_EQ(5u, navHudTrailCount());
    EXPECT_EQ(80, navHudTrailPointAt(navHudTrailCount() - 1)->eastM);
    // spacing doubled: a 5 m step is not recorded
    feedFix(85.0f, 0);
    EXPECT_EQ(5u, navHudTrailCount());
    // but a 20 m step is
    feedFix(100.0f, 0);
    EXPECT_EQ(6u, navHudTrailCount());
}

TEST_F(NavHudTest, RescueTrailPointsAreFlagged)
{
    arm();
    feedFix(0, 0);
    feedFix(50, 0, 1.0f, true);
    ASSERT_EQ(2u, navHudTrailCount());
    EXPECT_EQ(0, navHudTrailPointAt(0)->flags & NAV_HUD_TRAIL_FLAG_RESCUE);
    EXPECT_EQ(NAV_HUD_TRAIL_FLAG_RESCUE, navHudTrailPointAt(1)->flags & NAV_HUD_TRAIL_FLAG_RESCUE);
}

TEST_F(NavHudTest, LongDistancePointsAreClamped)
{
    arm();
    feedFix(0, 0);
    feedFix(40000, 0, 2.0f);       // rejected: implausible jump
    feedFix(40000, 0, 2.0f);       // second consistent sample: accepted
    ASSERT_GE(navHudTrailCount(), 2u);
    EXPECT_EQ(INT16_MAX, navHudTrailPointAt(navHudTrailCount() - 1)->eastM);
}

//
// GPS glitch handling
//

TEST_F(NavHudTest, SingleSampleGlitchIsRejected)
{
    arm();
    feedFix(0, 0);
    feedFix(10, 0);
    const navHudState_t *state = update();
    EXPECT_FALSE(state->gpsGlitch);

    feedFix(5000, 0);              // 5 km jump in 0.1 s: impossible
    state = update();
    EXPECT_TRUE(state->gpsGlitch);
    // craft position still reflects the last accepted fix
    EXPECT_NEAR(10.0f * 100.0f, state->craftPosCm.x, 1.0f);

    feedFix(12, 0);                // back to plausible positions
    state = update();
    EXPECT_FALSE(state->gpsGlitch);
    EXPECT_NEAR(12.0f * 100.0f, state->craftPosCm.x, 1.0f);
}

TEST_F(NavHudTest, ConsistentJumpIsAcceptedAsReacquisition)
{
    arm();
    feedFix(0, 0);
    feedFix(5000, 0);              // rejected at first
    EXPECT_TRUE(update()->gpsGlitch);
    feedFix(5005, 0);              // consistent with the "glitch": it was real
    const navHudState_t *state = update();
    EXPECT_FALSE(state->gpsGlitch);
    EXPECT_NEAR(5005.0f * 100.0f, state->craftPosCm.x, 100.0f);
}

//
// state assembly
//

TEST_F(NavHudTest, BasicStateAssembly)
{
    arm();
    attitude.values.yaw = 900;     // 90 degrees
    gpsSol.groundSpeed = 1000;     // 10 m/s
    gpsSol.groundCourse = 900;
    feedFix(300, 400);             // 500 m from home, bearing home = 216.9 deg
    const navHudState_t *state = update();

    EXPECT_TRUE(state->gpsValid);
    EXPECT_TRUE(state->homeValid);
    EXPECT_TRUE(state->positionValid);
    EXPECT_EQ(90, state->headingDeg);
    EXPECT_NEAR(50000, state->distanceToHomeCm, 100);
    EXPECT_NEAR(217, state->bearingToHomeDeg, 1);
    EXPECT_EQ(18, state->satellites);
    EXPECT_EQ(90, state->hdop);
    EXPECT_EQ(NAV_HUD_PHASE_IDLE, state->phase);
}

TEST_F(NavHudTest, EtaFromClosingSpeed)
{
    arm();
    // fly towards home at a steady 2 m/s from 500 m out
    for (int i = 0; i < 20; i++) {
        feedFix(500.0f - i * 2.0f, 0, 1.0f);
    }
    const navHudState_t *state = update();
    ASSERT_TRUE(state->etaValid);
    // 462 m remaining at 2 m/s closing speed -> ~231 s
    EXPECT_NEAR(231, state->etaSeconds, 10);
    EXPECT_EQ(1, state->trackTrend);
}

TEST_F(NavHudTest, HeadingWraparoundError)
{
    arm();
    attitude.values.yaw = 3500;    // heading 350
    feedFix(-100, 0);              // home is due East of the craft: bearing 90
    const navHudState_t *state = update();
    EXPECT_EQ(90, state->desiredHeadingDeg);
    EXPECT_EQ(100, state->headingErrorDeg);  // shortest turn is +100, right
}

TEST_F(NavHudTest, RescuePhaseMappingAndRouteCapture)
{
    arm();
    feedFix(300, 400);
    update();

    // engage rescue at (300, 400)
    flightModeFlags |= GPS_RESCUE_MODE;
    testRescuePhase = RESCUE_FLY_HOME;
    testTargetAltitudeCm = 4500;
    testTargetVelocityCmS = 1500;
    testDescentDistanceCm = 10000;
    const navHudState_t *state = update();

    EXPECT_TRUE(state->rescueActive);
    EXPECT_EQ(NAV_HUD_PHASE_RETURN, state->phase);
    EXPECT_TRUE(state->routeValid);
    EXPECT_NEAR(30000, state->rescueStartCm.x, 100);
    EXPECT_NEAR(40000, state->rescueStartCm.y, 100);
    EXPECT_EQ(4500, state->targetAltitudeCm);
    EXPECT_EQ(1500, state->targetSpeedCmS);
    EXPECT_EQ(10000, state->descentDistanceCm);
    EXPECT_EQ(0, state->crossTrackCm);  // on the planned line at capture

    // drift off the planned line: cross-track error appears
    feedFix(320, 380);
    state = update();
    EXPECT_NE(0, state->crossTrackCm);

    testRescuePhase = RESCUE_LANDING;
    state = update();
    EXPECT_EQ(NAV_HUD_PHASE_LAND, state->phase);

    testRescuePhase = RESCUE_ABORT;
    state = update();
    EXPECT_EQ(NAV_HUD_PHASE_ABORTED, state->phase);
}

TEST_F(NavHudTest, TrailResetsWhenHomeMoves)
{
    arm();
    feedFix(0, 0);
    feedFix(50, 0, 1.0f);
    EXPECT_EQ(2u, navHudTrailCount());

    GPS_home_llh.lat += 100000;    // home re-established elsewhere
    update();
    EXPECT_EQ(0u, navHudTrailCount());
}

TEST_F(NavHudTest, TrailResetsOnArming)
{
    arm();
    feedFix(0, 0);
    feedFix(50, 0, 1.0f);
    EXPECT_EQ(2u, navHudTrailCount());

    armingFlags = 0;
    update();
    armingFlags |= ARMED;          // new flight
    update();
    EXPECT_EQ(0u, navHudTrailCount());
}

TEST_F(NavHudTest, StaleWithoutFreshFixes)
{
    arm();
    feedFix(100, 100);
    const navHudState_t *state = update();
    EXPECT_FALSE(state->stale);

    testMicros += 5 * 1000 * 1000; // 5 s with no new fix
    state = update();
    EXPECT_TRUE(state->stale);
    EXPECT_FALSE(state->positionValid);
}

TEST_F(NavHudTest, ModeOffDisablesEverything)
{
    navHudConfigMutable()->mode = NAV_HUD_MODE_OFF;
    arm();
    gpsSol.llh.lat = 1000000;   // ~111 m North of home
    navHudOnGpsNewData();       // production entry point: gated on mode
    const navHudState_t *state = update();
    EXPECT_FALSE(state->positionValid);
    EXPECT_EQ(0u, navHudTrailCount());
}

//
// renderer
//

class NavHudRenderTest : public NavHudTest {
protected:
    static const int SCREEN_COLS = 53;
    static const int SCREEN_ROWS = 20;
    uint8_t screen[20][53];
    displayPort_t displayPort;
    char buff[OSD_ELEMENT_BUFFER_LENGTH];

    void SetUp() override
    {
        NavHudTest::SetUp();
        memset(screen, ' ', sizeof(screen));
        memset(&displayPort, 0, sizeof(displayPort));
        displayPort.cols = SCREEN_COLS;
        displayPort.rows = SCREEN_ROWS;
    }

    // mimic osdDrawSingleElement/osdDisplayActiveElement for one element
    void renderElement(void (*drawFn)(osdElementParms_t *), uint8_t posX, uint8_t posY)
    {
        testMicros += 60000; // let the internal state refresh run
        osdElementParms_t element;
        int guard = 64;
        do {
            memset(&element, 0, sizeof(element));
            element.item = OSD_NAV_HUD;
            element.elemPosX = posX;
            element.elemPosY = posY;
            element.type = OSD_ELEMENT_TYPE_1;
            element.buff = buff;
            element.osdDisplayPort = &displayPort;
            element.drawElement = true;
            element.rendered = true;
            element.attr = DISPLAYPORT_SEVERITY_NORMAL;
            buff[0] = '\0';

            drawFn(&element);

            if (element.drawElement) {
                const int x = (element.elemPosX + element.elemOffsetX) & 0xFF;
                const int y = (element.elemPosY + element.elemOffsetY) & 0xFF;
                for (int i = 0; buff[i] && (x + i) < SCREEN_COLS; i++) {
                    if (y >= 0 && y < SCREEN_ROWS && (x + i) >= 0) {
                        screen[y][x + i] = buff[i];
                    }
                }
            }
        } while (!element.rendered && --guard > 0);
        ASSERT_GT(guard, 0) << "element never finished rendering";
    }

    bool screenContains(const char *text)
    {
        const size_t len = strlen(text);
        for (int y = 0; y < SCREEN_ROWS; y++) {
            for (int x = 0; x + (int)len <= SCREEN_COLS; x++) {
                if (memcmp(&screen[y][x], text, len) == 0) {
                    return true;
                }
            }
        }
        return false;
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

    bool hasGlyphInRange(uint8_t first, uint8_t last)
    {
        for (int y = 0; y < SCREEN_ROWS; y++) {
            for (int x = 0; x < SCREEN_COLS; x++) {
                if (screen[y][x] >= first && screen[y][x] <= last) {
                    return true;
                }
            }
        }
        return false;
    }
};

TEST_F(NavHudRenderTest, NormalFlightMapRenders)
{
    arm();
    attitude.values.yaw = 450;
    feedFix(0, 0);
    feedFix(30, 0, 1.0f);
    feedFix(150, 200, 2.5f);

    renderElement(osdElementNavHud, 1, 1);

    // border drawn (19x9 standard map)
    EXPECT_EQ(screen[1][1], SYM_STICK_OVERLAY_HORIZONTAL);
    EXPECT_EQ(screen[1][19], SYM_STICK_OVERLAY_HORIZONTAL);
    EXPECT_EQ(screen[9][1], SYM_STICK_OVERLAY_HORIZONTAL);
    EXPECT_EQ(screen[5][1], SYM_STICK_OVERLAY_VERTICAL);
    // top-border cue points home once home is known
    EXPECT_EQ(screen[1][2], SYM_HOMEFLAG);
    EXPECT_TRUE(screen[1][3] >= SYM_ARROW_SOUTH && screen[1][3] <= SYM_ARROW_16);
    // home flag appears in the top cue, map marker, and footer distance prefix
    EXPECT_EQ(3, countGlyph(SYM_HOMEFLAG));
    EXPECT_EQ(1, countGlyph(SYM_ALTITUDE));
    EXPECT_TRUE(hasGlyphInRange(SYM_ARROW_SOUTH, SYM_ARROW_16));
    // footer shows distance to home (250 m -> "250M" via the stub formatter)
    EXPECT_TRUE(screenContains("250"));
    // satellite count stays in the top border even during normal/armed rendering
    EXPECT_TRUE(screenContains("18"));
}

TEST_F(NavHudRenderTest, AcquiringGpsPanelBeforeFix)
{
    // powered up, no fix yet: quiet acquisition panel, sats stay in the border
    gpsSol.numSat = 5;
    gpsSol.dop.hdop = 320;
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("WAIT GPS"));
    EXPECT_FALSE(screenContains("GPS SEEK"));
    EXPECT_FALSE(screenContains("SAT"));
    EXPECT_TRUE(screenContains(" 5"));
    EXPECT_EQ(0, countGlyph(SYM_PB_FULL));
    EXPECT_EQ(0, countGlyph(SYM_PB_EMPTY));
}

TEST_F(NavHudRenderTest, ArmPromptAfterFixBeforeHome)
{
    stateFlags |= GPS_FIX;   // fix, but no GPS_FIX_HOME yet
    gpsSol.numSat = 14;
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("GPS LOCKED"));
    EXPECT_TRUE(screenContains("ARM TO SET HOME"));
    EXPECT_TRUE(screenContains("14"));
    EXPECT_FALSE(screenContains("SAT"));
    EXPECT_EQ(0, countGlyph(SYM_PB_EMPTY));
}

TEST_F(NavHudRenderTest, HeadingUpRotatesWithNose)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_HEADING_UP;
    arm();
    attitude.values.yaw = 900;     // nose East, hovering
    gpsSol.groundSpeed = 0;
    feedFix(0, 0);
    feedFix(0, 100, 1.0f);         // craft north of home: home is due South

    renderElement(osdElementNavHud, 1, 1);

    // nose East + home South: home appears 90 degrees right of up
    EXPECT_EQ(screen[1][2], SYM_HOMEFLAG);
    EXPECT_EQ(screen[1][3], osdGetDirectionSymbolFromHeading(90));

    // yaw the craft to face home: home swings to the top of the map
    attitude.values.yaw = 1800;
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(screen[1][3], osdGetDirectionSymbolFromHeading(0));
}

TEST_F(NavHudRenderTest, NearHomeCueShowsHomeIconOnly)
{
    arm();
    feedFix(0, 0);
    feedFix(0, 5, 1.0f); // within roughly 30 ft

    renderElement(osdElementNavHud, 1, 1);

    EXPECT_EQ(screen[1][2], SYM_HOMEFLAG);
    EXPECT_EQ(screen[1][3], SYM_STICK_OVERLAY_HORIZONTAL);
}

TEST_F(NavHudRenderTest, RescueExpandsAndShowsPhaseAndRoute)
{
    navHudConfigMutable()->rescueExpand = 1;
    arm();
    feedFix(300, 400);
    update();

    flightModeFlags |= GPS_RESCUE_MODE;
    testRescuePhase = RESCUE_FLY_HOME;
    testTargetAltitudeCm = 4500;
    testTargetVelocityCmS = 1500;
    testDescentDistanceCm = 10000;
    gpsSol.groundSpeed = 1400;
    testAltitudeCm = 4200;
    feedFix(280, 380);

    renderElement(osdElementNavHud, 1, 1);

    // expanded from STANDARD (19) to FULL (27) while rescue is active
    EXPECT_EQ(screen[1][27], SYM_STICK_OVERLAY_HORIZONTAL);
    // phase banner
    EXPECT_TRUE(screenContains("RTH RETURN"));
    // guidance to home draws as a dashed stroke of AH-bar glyphs
    int strokeCells = 0;
    for (int g = 0; g < 9; g++) {
        strokeCells += countGlyph(SYM_AH_BAR9_0 + g);
    }
    EXPECT_GE(strokeCells, 2);
}

TEST_F(NavHudRenderTest, RescueFailureBanner)
{
    arm();
    feedFix(300, 400);
    update();
    flightModeFlags |= GPS_RESCUE_MODE;
    testRescuePhase = RESCUE_FLY_HOME;
    testRescueIsOK = false;
    testRescueFailure = RESCUE_GPSLOST;

    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("RTH GPS LOST"));
}



TEST_F(NavHudRenderTest, TopSpeedCenteredInBorder)
{
    arm();
    gpsSol.groundSpeed = 2000;     // stub converts cm/s -> /100, so "20"
    feedFix(150, 200, 2.5f);
    renderElement(osdElementNavHud, 1, 1);

    ASSERT_EQ(1, countGlyph(SYM_SPEED));
    // find the dial icon in the top border row and check the value behind it
    int x = -1;
    for (int i = 0; i < SCREEN_COLS; i++) {
        if (screen[1][i] == SYM_SPEED) {
            x = i;
            break;
        }
    }
    ASSERT_GE(x, 5);               // centred, clear of the home cue
    EXPECT_EQ('2', screen[1][x + 1]);
    EXPECT_EQ('0', screen[1][x + 2]);
    EXPECT_EQ('K', screen[1][x + 3]);      // unit glyph from the stub
    EXPECT_EQ(' ', screen[1][x - 1]);      // breathing space in the border line
    EXPECT_EQ(' ', screen[1][x + 4]);
}

TEST_F(NavHudRenderTest, TopSpeedHiddenWhenDisabled)
{
    navHudConfigMutable()->showSpeed = 0;
    arm();
    gpsSol.groundSpeed = 2000;
    feedFix(150, 200, 2.5f);
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(0, countGlyph(SYM_SPEED));
}

TEST_F(NavHudRenderTest, CompactTopBorderPrioritisesSpeedOverSats)
{
    navHudConfigMutable()->mode = NAV_HUD_MODE_COMPACT;
    arm();
    gpsSol.groundSpeed = 2000;
    feedFix(150, 200, 2.5f);
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(1, countGlyph(SYM_SPEED));
    EXPECT_EQ(0, countGlyph(SYM_SAT_L));   // no room on a 13-wide border
}

TEST_F(NavHudRenderTest, HoldBannerWithBracketsAndAnchor)
{
    navHudConfigMutable()->breadcrumbs = 0;   // keep sprite-dot asserts unambiguous
    arm();
    feedFix(80, 60, 2.5f);
    flightModeFlags |= POS_HOLD_MODE | ALT_HOLD_MODE;
    testTargetPosValid = true;
    // anchor a couple of cells away from the craft, as when wind drifts the
    // quad off its hold point (same cell would let the craft arrow cover it)
    testTargetPos = { { 60.0f * 100.0f, 45.0f * 100.0f } };

    renderElement(osdElementNavHud, 1, 1);

    // both modes on a 19-wide map: combined label with converging brackets
    EXPECT_TRUE(screenContains("POS+ALT HOLD"));
    EXPECT_GE(countGlyph(SYM_AH_RIGHT), 1);
    EXPECT_GE(countGlyph(SYM_AH_LEFT), 1);
    // anchor marker plus its shimmer dots on the map
    EXPECT_GE(countGlyph(SYM_STICK_OVERLAY_CENTER), 1);
    const int shimmer = countGlyph(SYM_STICK_OVERLAY_SPRITE_HIGH)
        + countGlyph(SYM_STICK_OVERLAY_SPRITE_MID)
        + countGlyph(SYM_STICK_OVERLAY_SPRITE_LOW);
    EXPECT_EQ(2, shimmer);
}

TEST_F(NavHudRenderTest, MagAllowsHeadingUpAtHover)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_HEADING_UP;
    testMagCalibrated = true;      // compass wired, detected and calibrated
    arm();
    attitude.values.yaw = 900;     // hovering, nose East
    gpsSol.groundSpeed = 0;
    feedFix(0, 0);
    feedFix(0, 100, 1.0f);         // craft north of home

    renderElement(osdElementNavHud, 1, 1);

    // the map rotates even at standstill: home (due South of the craft)
    // appears 90 degrees right of straight-up in the border cue
    EXPECT_EQ(screen[1][2], SYM_HOMEFLAG);
    EXPECT_EQ(screen[1][3], osdGetDirectionSymbolFromHeading(90));
}

TEST_F(NavHudRenderTest, OffMapHomeShowsEdgeDirectionArrow)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    navHudConfigMutable()->autoZoom = 0;
    navHudConfigMutable()->fixedScaleM = 100;   // 100 m wide map, home 500 m away
    navHudConfigMutable()->breadcrumbs = 0;
    navHudConfigMutable()->homeLine = 0;
    arm();
    attitude.values.yaw = 900;
    feedFix(0, 0);
    feedFix(250, 0, 2.5f);
    feedFix(500, 0, 2.5f);         // craft 500 m East of home: home due West, off-map

    renderElement(osdElementNavHud, 1, 1);

    // no in-map home flag (only the border cue and the footer prefix carry one)
    EXPECT_EQ(2, countGlyph(SYM_HOMEFLAG));
    // a West-pointing arrow sits pinned at the map edge
    EXPECT_GE(countGlyph(osdGetDirectionSymbolFromHeading(270)), 1);
}

TEST_F(NavHudRenderTest, TrailConnectsFlownPathAndIgnoresHover)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->homeLine = 0;
    arm();
    feedFix(0, 0);
    for (int i = 1; i <= 10; i++) {
        feedFix(i * 10.0f, 0, 1.0f);
    }

    renderElement(osdElementNavHud, 1, 1);
    int flown = 0;
    for (int g = 0; g < 9; g++) {
        flown += countGlyph(SYM_AH_BAR9_0 + g);
    }
    flown += countGlyph(SYM_STICK_OVERLAY_SPRITE_MID)
           + countGlyph(SYM_STICK_OVERLAY_SPRITE_HIGH)
           + countGlyph(SYM_STICK_OVERLAY_SPRITE_LOW);
    EXPECT_GE(flown, 3);   // the path renders as a continuous line of ink

    // hovering: plenty of time passes, no distance is flown - the line must
    // not grow or sprout a cluster
    for (int i = 0; i < 20; i++) {
        feedFix(100.0f, 0, 1.0f);
    }
    renderElement(osdElementNavHud, 1, 1);
    int hovered = 0;
    for (int g = 0; g < 9; g++) {
        hovered += countGlyph(SYM_AH_BAR9_0 + g);
    }
    hovered += countGlyph(SYM_STICK_OVERLAY_SPRITE_MID)
             + countGlyph(SYM_STICK_OVERLAY_SPRITE_HIGH)
             + countGlyph(SYM_STICK_OVERLAY_SPRITE_LOW);
    EXPECT_EQ(flown, hovered);
}

TEST_F(NavHudRenderTest, TrailUsesSubCellLineGlyphs)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    arm();
    feedFix(0, 0);
    // a shallow diagonal: the line crosses cell rows gradually, so sub-cell
    // smoothing must place dots at high/low positions inside cells, not just
    // stair-step with centre dots
    for (int i = 1; i <= 8; i++) {
        feedFix(i * 40.0f, i * 10.0f, 1.0f);
    }

    renderElement(osdElementNavHud, 1, 1);

    int distinctBarHeights = 0;
    for (int g = 0; g < 9; g++) {
        if (countGlyph(SYM_AH_BAR9_0 + g) > 0) {
            distinctBarHeights++;
        }
    }
    EXPECT_GE(distinctBarHeights, 2);
}

TEST_F(NavHudRenderTest, StrokeIsSingleCellThick)
{
    // regression: a gently descending line must never draw doubled ink where
    // it rides a cell boundary - exactly one glyph per column
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    navHudConfigMutable()->homeLine = 0;
    navHudConfigMutable()->rangeRing = 0;
    arm();
    feedFix(0, 0);
    for (int i = 1; i <= 10; i++) {
        feedFix(i * 20.0f, i * -3.0f, 1.0f);   // shallow descent on the map
    }

    renderElement(osdElementNavHud, 1, 1);

    int columnsWithInk = 0;
    for (int x = 0; x < SCREEN_COLS; x++) {
        int ink = 0;
        for (int y = 0; y < SCREEN_ROWS; y++) {
            const uint8_t g = (uint8_t)screen[y][x];
            if ((g >= SYM_AH_BAR9_0 && g <= SYM_AH_BAR9_0 + 8)) {
                ink++;
            }
        }
        EXPECT_LE(ink, 1) << "doubled stroke ink in column " << x;
        if (ink > 0) {
            columnsWithInk++;
        }
    }
    EXPECT_GE(columnsWithInk, 3);
}

extern bool testWaypointSwitch;

TEST_F(NavHudRenderTest, WaypointsDropOnSwitchEdgeAndRender)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    arm();
    feedFix(0, 0);
    feedFix(50, 0, 1.0f);

    testWaypointSwitch = true;    // press: candidate captured...
    update();
    EXPECT_EQ(0u, navHudWaypointCount());
    testWaypointSwitch = false;   // ...tap released: waypoint 1 committed
    update();
    EXPECT_EQ(1u, navHudWaypointCount());

    feedFix(120, 40, 1.0f);
    testWaypointSwitch = true;    // second tap: waypoint 2
    update();
    testWaypointSwitch = false;
    update();

    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(2u, navHudWaypointCount());
    EXPECT_GE(countGlyph('1'), 1);
    EXPECT_GE(countGlyph('2'), 1);

    // holding the switch ~1.5 s deletes the last waypoint instead of adding
    testWaypointSwitch = true;
    for (int i = 0; i < 30; i++) {   // 30 x 60 ms = 1.8 s held
        update();
    }
    EXPECT_EQ(1u, navHudWaypointCount());
    testWaypointSwitch = false;
    update();
    EXPECT_EQ(1u, navHudWaypointCount());   // release adds nothing after an undo
}

// The hold-to-delete gesture must work even with a full route (the "room to
// add?" check belongs on the drop, not the gesture) and must confirm on screen.
TEST_F(NavHudRenderTest, HoldToDeleteWorksAtMaxWaypointsAndConfirms)
{
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    arm();
    feedFix(0, 0);
    feedFix(5, 0, 1.0f);
    for (int i = 0; i < NAV_HUD_WAYPOINT_MAX; i++) {
        feedFix(20.0f + i * 20.0f, 0.0f, 1.0f);
        testWaypointSwitch = true;
        update();
        testWaypointSwitch = false;
        update();
    }
    ASSERT_EQ((unsigned)NAV_HUD_WAYPOINT_MAX, navHudWaypointCount());

    // hold ~1.8 s on a FULL route: the delete still fires
    testWaypointSwitch = true;
    for (int i = 0; i < 30; i++) {
        update();
    }
    EXPECT_EQ((unsigned)(NAV_HUD_WAYPOINT_MAX - 1), navHudWaypointCount());

    // and the pilot gets an on-screen confirmation
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("DELETED"));

    testWaypointSwitch = false;
    update();
    EXPECT_EQ((unsigned)(NAV_HUD_WAYPOINT_MAX - 1), navHudWaypointCount());   // release adds nothing
}

TEST_F(NavHudRenderTest, FirstWaypointDropShowsOneTimeHint)
{
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    arm();
    feedFix(0, 0);
    feedFix(50, 0, 1.0f);

    testWaypointSwitch = true;
    update();
    testWaypointSwitch = false;
    update();
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("HOLD=DEL"));   // teachable moment on drop 1

    // hint expires...
    for (int i = 0; i < 70; i++) {
        update();   // 70 x 60 ms > 3.5 s
    }
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_FALSE(screenContains("HOLD=DEL"));

    // ...and never returns for later drops
    feedFix(120, 0, 1.0f);
    testWaypointSwitch = true;
    update();
    testWaypointSwitch = false;
    update();
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_FALSE(screenContains("HOLD=DEL"));
}

TEST_F(NavHudRenderTest, WaypointSessionExpandsTheMap)
{
    navHudConfigMutable()->mode = NAV_HUD_MODE_COMPACT;
    navHudConfigMutable()->rescueExpand = 1;
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    arm();
    feedFix(0, 0);
    renderElement(osdElementNavHud, 1, 1);
    // compact: 14 columns -> rightmost border at screen column 14
    EXPECT_EQ(SYM_STICK_OVERLAY_HORIZONTAL, (uint8_t)screen[1][14]);
    EXPECT_NE(SYM_STICK_OVERLAY_HORIZONTAL, (uint8_t)screen[1][19]);

    feedFix(50, 0, 1.0f);
    testWaypointSwitch = true;
    update();
    testWaypointSwitch = false;
    update();
    renderElement(osdElementNavHud, 1, 1);
    // a waypoint session grows the map one tier (standard: 19 columns)
    EXPECT_EQ(SYM_STICK_OVERLAY_HORIZONTAL, (uint8_t)screen[1][19]);
}

TEST_F(NavHudRenderTest, WaypointMarkersCarryAltitudeTags)
{
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    navHudConfigMutable()->mission3D = 1;
    navHudConfigMutable()->breadcrumbs = 0;
    navHudConfigMutable()->homeLine = 0;
    navHudConfigMutable()->rangeRing = 0;
    arm();
    feedFix(0, 0);
    testAltitudeCm = 4500;         // waypoint dropped at 45 m
    feedFix(60, 0, 1.0f);
    testWaypointSwitch = true;
    update();
    testWaypointSwitch = false;
    update();

    feedFix(20, 0, 1.0f);          // move away so the craft marker doesn't cover the digit
    renderElement(osdElementNavHud, 1, 1);
    // near the fit edge the tag draws on the left: "45:1"
    EXPECT_TRUE(screenContains("1:45") || screenContains("45:1"));
}

class NavMissionTest : public NavHudRenderTest {
protected:
    uint32_t missionTimeUs = 1000000;

    void SetUp() override
    {
        NavHudRenderTest::SetUp();
        gpsRescueConfigMutable()->groundSpeedCmS = 750;
        gpsRescueConfigMutable()->yawP = 20;
        testAltitudeCm = 2000;   // airborne: missions refuse to engage on the ground
    }

    void dropWaypoint(float eastM, float northM)
    {
        feedFix(eastM, northM, 2.0f);
        testWaypointSwitch = true;
        update();
        testWaypointSwitch = false;
        update();
    }

    void missionTick(uint32_t advanceMs)
    {
        missionTimeUs += advanceMs * 1000;
        navMissionUpdate(missionTimeUs);
    }
};

TEST_F(NavMissionTest, RotatesFirstThenMarchesCarrotAtRescueSpeed)
{
    arm();
    feedFix(0, 0);
    dropWaypoint(100, 0);          // waypoint due East
    ASSERT_EQ(1u, navHudWaypointCount());

    setCraftLL(0, 0);
    attitude.values.yaw = 0;       // nose North: 90 degrees off the waypoint bearing
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_TRUE(navMissionIsControlling());

    // rotate phase: rescue's sign convention - heading 0, bearing 90 gives
    // errorAngle -90, so the yaw rate is negative (capped at -180) - while
    // the carrot stays parked at the craft
    missionTick(100);
    EXPECT_NEAR(-180.0f, navMissionGetYawRateDps(), 1.0f);
    EXPECT_NEAR(0, testMissionTarget.lon, 30);

    // aligned: the carrot accelerates East at 250 cm/s^2 toward
    // gps_rescue_ground_speed - no instant full-speed leap. (The rotate tick
    // above contributes one slew step: atan2_approx puts the exact-90-degree
    // error a hair inside the 0.25 carve floor.)
    attitude.values.yaw = 900;
    for (int i = 0; i < 10; i++) {
        missionTick(100);          // 1 s of ramp: ~162 cm of carrot travel
    }
    EXPECT_NEAR(162.5f / EARTH_ANGLE_TO_CM, testMissionTarget.lon, 40);
    EXPECT_NEAR(0.0f, navMissionGetYawRateDps(), 5.0f);
    EXPECT_NEAR(275.0f, navMissionCarrotSpeedForTest(), 30.0f);

    // the craft is pinned in this fixture, so the carrot runs out to the
    // speed-proportional lead cap (v30: 1.2 s at 750 cm/s = 9 m - the 2 s
    // lead saturated the position P into a 10-14 m/s chase) and waits there
    for (int i = 0; i < 30; i++) {
        missionTick(100);
    }
    EXPECT_NEAR(900.0f / EARTH_ANGLE_TO_CM, testMissionTarget.lon, 60);
    EXPECT_NEAR(750.0f, navMissionCarrotSpeedForTest(), 1.0f);
}

TEST_F(NavMissionTest, AdvancesThroughWaypointsAndParksAtLast)
{
    arm();
    feedFix(0, 0);
    dropWaypoint(100, 0);
    dropWaypoint(100, 100);

    setCraftLL(0, 0);
    attitude.values.yaw = 900;
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    // arrive at waypoint 1: next leg begins
    setCraftLL(99, 0);
    missionTick(100);
    EXPECT_EQ(1u, navMissionActiveIndex());

    // leg 2 runs North; heading East vs bearing ~0 gives a positive error
    // in rescue's convention, so the yaw rate is positive
    missionTick(100);
    EXPECT_GT(navMissionGetYawRateDps(), 50.0f);

    // arriving at the final waypoint parks exactly on it
    attitude.values.yaw = 0;
    setCraftLL(100, 99);
    missionTick(100);
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_TRUE(navMissionIsControlling());   // still holding at the waypoint
    EXPECT_NEAR(10000.0f / EARTH_ANGLE_TO_CM, testMissionTarget.lat, 3);
    EXPECT_EQ(0.0f, navMissionGetYawRateDps());

    testMissionSwitch = false;
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_IDLE, navMissionGetPhase());
    EXPECT_FALSE(navMissionIsControlling());
}

// Regression for the field bug: fly a mission, interrupt it on a late leg
// (so the retained index is not reset by the DONE/overflow guard), move the
// craft far away, re-engage - it must restart at waypoint 1, not fly straight
// to the stale late waypoint.
TEST_F(NavMissionTest, ReEngageAfterLateAbortRestartsAtWaypointOne)
{
    navHudConfigMutable()->mission3D = 0;   // 2D: arrival is pure horizontal distance
    arm();
    feedFix(0, 0);
    dropWaypoint(50, 0);           // wp1
    dropWaypoint(100, 0);          // wp2
    dropWaypoint(150, 0);          // wp3 (final)
    ASSERT_EQ(3u, navHudWaypointCount());

    setCraftLL(0, 0);
    attitude.values.yaw = 900;     // nose East, onto the first leg
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    // reach the FINAL leg without completing it
    setCraftLL(50, 0);
    missionTick(100);
    ASSERT_EQ(1u, navMissionActiveIndex());
    setCraftLL(100, 0);
    missionTick(100);
    ASSERT_EQ(2u, navMissionActiveIndex());   // on the last leg, not yet DONE

    // deliberate stick grab aborts here, leaving the index parked at 2
    testStickDeflection = 0.3f;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
    testStickDeflection = 0.0f;

    // switch off, fly ~400 m away, switch back on
    testMissionSwitch = false;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_IDLE, navMissionGetPhase());
    setCraftLL(-400, 0);
    testMissionSwitch = true;
    missionTick(10);

    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_EQ(0u, navMissionActiveIndex());    // restarts at wp1, NOT the stale wp3
    EXPECT_EQ(3u, navHudWaypointCount());       // the route itself survived the cycle
}

// The counterpart: cancel mid-mission to watch, stay roughly put, flip back on
// - the mission RESUMES from the current waypoint (continue to wp4 after
// cancelling past wp3) because the craft is still near where it was interrupted.
TEST_F(NavMissionTest, ReEngageWhileStillNearbyResumesFromCurrentWaypoint)
{
    navHudConfigMutable()->mission3D = 0;
    arm();
    feedFix(0, 0);
    dropWaypoint(50, 0);
    dropWaypoint(100, 0);
    dropWaypoint(150, 0);

    setCraftLL(0, 0);
    attitude.values.yaw = 900;
    testMissionSwitch = true;
    missionTick(10);
    setCraftLL(50, 0);
    missionTick(100);
    ASSERT_EQ(1u, navMissionActiveIndex());   // passed wp1, heading to wp2

    // cancel to watch; the craft only drifts a few metres
    testMissionSwitch = false;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_IDLE, navMissionGetPhase());
    setCraftLL(58, 0);            // ~8 m, well inside the resume radius
    testMissionSwitch = true;
    missionTick(10);

    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_EQ(1u, navMissionActiveIndex());   // resumed at wp2, did NOT restart at wp1
}

// A completed mission re-engaged from the switch must also restart at wp1
// (guards the DONE/overflow reset that the old code relied on exclusively).
TEST_F(NavMissionTest, ReEngageAfterCompletionRestartsAtWaypointOne)
{
    navHudConfigMutable()->mission3D = 0;
    arm();
    feedFix(0, 0);
    dropWaypoint(100, 0);
    dropWaypoint(100, 100);

    setCraftLL(0, 0);
    attitude.values.yaw = 900;
    testMissionSwitch = true;
    missionTick(10);
    setCraftLL(100, 0);
    missionTick(100);
    attitude.values.yaw = 0;
    setCraftLL(100, 100);
    missionTick(100);
    ASSERT_EQ(NAV_MISSION_DONE, navMissionGetPhase());

    // cancel and re-engage while STILL PARKED on the final waypoint: a
    // completed mission restarts from scratch (its index has reached the
    // count), so proximity does not resume it - it goes back to waypoint 1.
    testMissionSwitch = false;
    missionTick(10);
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_EQ(0u, navMissionActiveIndex());
}

// The dropped route persists across a disarm/arm (battery swap on the same
// spot); only the per-flight trail resets on the arm edge.
TEST_F(NavMissionTest, WaypointsPersistAcrossDisarmAndArm)
{
    arm();
    feedFix(0, 0);
    dropWaypoint(50, 0);
    dropWaypoint(100, 0);
    ASSERT_EQ(2u, navHudWaypointCount());

    armingFlags = 0;
    update();
    armingFlags |= ARMED;          // re-arm on the same spot
    update();

    EXPECT_EQ(2u, navHudWaypointCount());   // route survived
    EXPECT_EQ(0u, navHudTrailCount());      // trail did not
}

// Home jitter on re-arm keeps the route; a genuine relocation to a new field
// clears it (waypoints are stored relative to home).
TEST_F(NavMissionTest, HomeJitterKeepsRouteButRelocationClearsIt)
{
    arm();
    feedFix(0, 0);
    dropWaypoint(50, 0);
    ASSERT_EQ(1u, navHudWaypointCount());

    GPS_home_llh.lat += 500;       // ~5.6 m of GPS noise: under the 25 m guard
    update();
    EXPECT_EQ(1u, navHudWaypointCount());

    GPS_home_llh.lat += 100000;    // ~1.1 km: a genuinely new site
    update();
    EXPECT_EQ(0u, navHudWaypointCount());
}

TEST_F(NavMissionTest, SticksAbortAndOnlyASwitchCycleResumes)
{
    arm();
    feedFix(0, 0);
    dropWaypoint(100, 0);

    setCraftLL(0, 0);
    attitude.values.yaw = 900;
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    testStickDeflection = 0.2f;    // a deliberate grab, above the 0.15 threshold
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
    EXPECT_FALSE(navMissionIsControlling());
    EXPECT_EQ(0.0f, navMissionGetYawRateDps());

    // centring the sticks is not enough - the abort latches
    testStickDeflection = 0.0f;
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());

    // a deliberate switch cycle re-engages at the same waypoint
    testMissionSwitch = false;
    missionTick(10);
    testMissionSwitch = true;
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_EQ(0u, navMissionActiveIndex());
}

TEST_F(NavMissionTest, RefusesGeofenceBreachAndGroundEngagement)
{
    arm();
    feedFix(0, 0);
    for (int i = 1; i <= 10; i++) {
        feedFix(i * 250.0f, 0, 2.5f);   // walk out to 2.5 km
    }
    testWaypointSwitch = true;
    update();
    testWaypointSwitch = false;
    update();
    ASSERT_EQ(1u, navHudWaypointCount());

    testMissionSwitch = true;
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());   // beyond the 2 km fence

    // and separately: a mission never engages on the ground
    navMissionResetForTest();
    navHudInit();                  // full reset: also clears the persistent route
    feedFix(0, 0);
    dropWaypoint(100, 0);
    testAltitudeCm = 100;          // 1 m: not airborne
    setCraftLL(0, 0);
    testMissionSwitch = true;
    missionTick(10);
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
}

TEST_F(NavMissionTest, HudShowsMissionFooterAndGuidesToActiveWaypoint)
{
    navHudConfigMutable()->homeLine = 0;
    arm();
    feedFix(0, 0);
    dropWaypoint(150, 0);
    setCraftLL(0, 0);
    feedFix(0, 0, 1.0f);
    attitude.values.yaw = 900;
    testMissionSwitch = true;
    missionTick(10);
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    renderElement(osdElementNavHud, 1, 1);

    EXPECT_TRUE(screenContains("WP1/1"));
    // the dashed guide stroke runs toward the waypoint even with the home
    // line disabled
    int strokeCells = 0;
    for (int g = 0; g < 9; g++) {
        strokeCells += countGlyph(SYM_AH_BAR9_0 + g);
    }
    EXPECT_GE(strokeCells, 1);
}

TEST_F(NavHudRenderTest, HomeCenteredDefaultsSatsCornerAndZoomFloor)
{
    // factory defaults: north-up, home pinned at the centre
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    navHudConfigMutable()->center = NAV_HUD_CENTER_HOME;
    arm();
    attitude.values.yaw = 900;
    feedFix(0, 0);
    feedFix(30, 0, 1.0f);          // just 30 m out: fit alone would over-zoom

    renderElement(osdElementNavHud, 1, 1);
    // the map announces its scale when it first appears...
    EXPECT_TRUE(screenContains("150M"));

    // ...then goes quiet: scale is carried by the range ring (dotted circle
    // around home), not an always-on number
    for (int f = 0; f < 30; f++) {
        renderElement(osdElementNavHud, 1, 1);
    }
    EXPECT_FALSE(screenContains("150M"));
    EXPECT_GE(countGlyph('+'), 3);
    // satellite count lives in the top-left corner (home cue is redundant)
    EXPECT_EQ(screen[1][2], SYM_SAT_L);
    EXPECT_TRUE(screenContains("18"));
    // home flag at the centre plus the footer prefix - and no corner cue
    EXPECT_EQ(2, countGlyph(SYM_HOMEFLAG));
    // the craft is the directional arrow, pointing East on a non-rotating map
    EXPECT_GE(countGlyph(osdGetDirectionSymbolFromHeading(90)), 1);
}

TEST_F(NavHudRenderTest, ZoomChangePulsesScaleBrackets)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_NORTH_UP;
    arm();
    feedFix(0, 0);
    feedFix(30, 0, 1.0f);
    renderElement(osdElementNavHud, 1, 1);

    // fly far enough out that auto-zoom must step the scale
    for (int i = 2; i <= 12; i++) {
        feedFix(i * 30.0f, 0, 1.0f);
    }

    // during the flash window alternate frames double the brackets
    int doubledBracketFrames = 0;
    for (int f = 0; f < 6; f++) {
        renderElement(osdElementNavHud, 1, 1);
        if (countGlyph(SYM_AH_LEFT) >= 2) {
            doubledBracketFrames++;
        }
    }
    EXPECT_GE(doubledBracketFrames, 1);
    // after the flash expires the label settles back to single brackets
    for (int f = 0; f < 16; f++) {
        renderElement(osdElementNavHud, 1, 1);
    }
    EXPECT_EQ(countGlyph(SYM_AH_LEFT), 1);
}

TEST_F(NavHudRenderTest, SuspectMagFallsBackToCourseAndWarns)
{
    navHudConfigMutable()->orientation = NAV_HUD_ORIENTATION_HEADING_UP;
    testMagCalibrated = true;
    arm();
    attitude.values.yaw = 900;     // compass claims East...
    gpsSol.groundCourse = 0;       // ...while we demonstrably fly North
    gpsSol.groundSpeed = 700;
    feedFix(0, 0);
    feedFix(0, 100, 1.0f);         // craft north of home: home due South

    // sustained disagreement (>2 s at the snapshot rate) latches suspicion;
    // keep fixes flowing so the position never goes stale
    for (int i = 0; i < 45; i++) {
        feedFix(0, 100.0f + i, 0.1f);
        update();
    }
    EXPECT_TRUE(navHudGetState()->headingSuspect);

    renderElement(osdElementNavHud, 1, 1);
    // rotation now uses course (North-up motion): home South renders down,
    // not the 90-degrees-right the lying compass would produce
    EXPECT_EQ(screen[1][3], osdGetDirectionSymbolFromHeading(180));

    // the MAG? warning appears in the top border on its blink phase
    bool warned = false;
    for (int f = 0; f < 20 && !warned; f++) {
        renderElement(osdElementNavHud, 1, 1);
        warned = screenContains("MAG?");
    }
    EXPECT_TRUE(warned);
}

TEST_F(NavHudRenderTest, RescueExpansionAnchorsBottomPlacement)
{
    navHudConfigMutable()->mode = NAV_HUD_MODE_COMPACT;
    navHudConfigMutable()->rescueExpand = 1;
    arm();
    feedFix(300, 400, 2.5f);
    update();
    flightModeFlags |= GPS_RESCUE_MODE;
    testRescuePhase = RESCUE_FLY_HOME;

    // bottom-left placement: compact footprint is 7 map rows + 1 footer,
    // placed so its bottom edge sits on the last screen row (y=11..19)
    renderElement(osdElementNavHud, 1, 11);

    // expanded content must grow upward: nothing may be clipped below,
    // so the rescue banner (first footer) is still fully on screen
    EXPECT_TRUE(screenContains("RTH RETURN"));
    // and the map's top border appears above the original placement row
    bool borderAboveOriginal = false;
    for (int y = 0; y < 11; y++) {
        if (screen[y][1] == SYM_STICK_OVERLAY_HORIZONTAL) {
            borderAboveOriginal = true;
            break;
        }
    }
    EXPECT_TRUE(borderAboveOriginal);
}

TEST_F(NavHudRenderTest, HoldZoomsToAnchorNotHome)
{
    navHudConfigMutable()->center = NAV_HUD_CENTER_CRAFT;
    arm();
    feedFix(80, 60, 2.5f);         // 100 m out from home
    flightModeFlags |= POS_HOLD_MODE;
    testTargetPosValid = true;
    testTargetPos = { { 60.0f * 100.0f, 45.0f * 100.0f } };

    renderElement(osdElementNavHud, 1, 1);

    // auto-zoom frames the anchor neighbourhood (100 m width), not the
    // 100 m-away home point (which would force a 400 m map)
    EXPECT_TRUE(screenContains("75M"));
    EXPECT_FALSE(screenContains("400M"));
    // home is edge-pinned on the border rather than dropped
    EXPECT_GE(countGlyph(SYM_HOMEFLAG), 1);
}

TEST_F(NavHudRenderTest, AltHoldOnlyBanner)
{
    arm();
    feedFix(80, 60, 2.5f);
    flightModeFlags |= ALT_HOLD_MODE;
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_TRUE(screenContains("ALT HOLD"));
    EXPECT_FALSE(screenContains("POS"));
}

TEST_F(NavHudRenderTest, CraftArrowFallsBackToCogThenNeutral)
{
    navHudConfigMutable()->breadcrumbs = 0;
    navHudConfigMutable()->homeLine = 0;
    canUseGPSHeading = false;
    arm();
    attitude.values.yaw = 0;
    gpsSol.groundCourse = 900;     // COG 90 degrees
    gpsSol.groundSpeed = 300;      // fast enough for COG to be valid
    feedFix(150, 200, 2.5f);
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(1, countGlyph(osdGetDirectionSymbolFromHeading(90)));

    // stationary with no heading source at all: neutral position dot
    gpsSol.groundSpeed = 0;
    feedFix(150, 200, 1.0f);
    renderElement(osdElementNavHud, 1, 1);
    EXPECT_EQ(0, countGlyph(osdGetDirectionSymbolFromHeading(90)));
    EXPECT_GE(countGlyph(SYM_STICK_OVERLAY_CENTER), 1);
}

// STUBS

extern "C" {
    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;

    uint32_t testMicros = 0;
    rescuePhase_e testRescuePhase = RESCUE_IDLE;
    rescueFailureState_e testRescueFailure = RESCUE_HEALTHY;
    float testTargetAltitudeCm = 0;
    float testTargetVelocityCmS = 0;
    float testDescentDistanceCm = 0;
    bool testRescueIsOK = true;

    float testAltitudeCm = 0;
    float testVerticalVelCmS = 0;
    bool testTargetPosValid = false;
    vector2_t testTargetPos = {{ 0, 0 }};

    bool testMagCalibrated = false;

    uint32_t micros(void) { return testMicros; }

    bool canUseGPSHeading = true;
    bool sensors(uint32_t mask) { UNUSED(mask); return testMagCalibrated; }
    bool compassIsHealthy(void) { return testMagCalibrated; }
    bool gpsIsHealthy(void) { return true; }

    // matches the firmware conversion: 1e-7 degree steps to cm
    void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
    {
        distance->x = (to->lon - from->lon) * EARTH_ANGLE_TO_CM;
        distance->y = (to->lat - from->lat) * EARTH_ANGLE_TO_CM;
    }

    float GPS_cosLat = 1.0f;

    bool testFailsafeActive = false;
    bool failsafeIsActive(void) { return testFailsafeActive; }

    float testStickDeflection = 0.0f;
    float getRcDeflectionAbs(int axis) { UNUSED(axis); return testStickDeflection; }

    gpsLocation_t testMissionTarget;
    float testMissionSanityCm = 0.0f;
    int testMissionTargetSets = 0;
    void posControlSetMissionTarget(const gpsLocation_t *location, float sanityCheckDistanceCm)
    {
        testMissionTarget = *location;
        testMissionSanityCm = sanityCheckDistanceCm;
        testMissionTargetSets++;
    }

    int32_t getEstimatedAltitudeCm(void) { return (int32_t)testAltitudeCm; }
    float getAltitudeDerivative(void) { return testVerticalVelCmS; }

    rescuePhase_e gpsRescueGetPhase(void) { return testRescuePhase; }
    rescueFailureState_e gpsRescueGetFailure(void) { return testRescueFailure; }
    float gpsRescueGetTargetAltitudeCm(void) { return testTargetAltitudeCm; }
    float gpsRescueGetReturnAltitudeCm(void) { return testTargetAltitudeCm; }
    float gpsRescueGetTargetVelocityCmS(void) { return testTargetVelocityCmS; }
    float gpsRescueGetDescentDistanceCm(void) { return testDescentDistanceCm; }
    bool gpsRescueIsOK(void) { return testRescueIsOK; }
    bool gpsRescueIsConfigured(void) { return true; }
    bool gpsRescueIsAvailable(void) { return true; }

    bool autopilotGetTargetPositionEfCm(vector2_t *targetEfCm)
    {
        *targetEfCm = testTargetPos;
        return testTargetPosValid;
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

    void osdFormatDistanceString(char *ptr, int distance, char leadingSymbol)
    {
        char *p = ptr;
        if (leadingSymbol != SYM_NONE) {
            *p++ = leadingSymbol;
        }
        tfp_sprintf(p, "%dM", distance);
    }

    float osdGetMetersToSelectedUnit(int32_t meters) { return meters; }
    char osdGetMetersToSelectedUnitSymbol(void) { return 'M'; }
    int32_t osdGetSpeedToSelectedUnit(int32_t value) { return value / 100; }
    char osdGetSpeedToSelectedUnitSymbol(void) { return 'K'; }
}
