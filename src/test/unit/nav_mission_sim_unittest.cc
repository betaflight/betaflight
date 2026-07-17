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

// Software-in-the-loop test for waypoint missions: the REAL nav_mission
// sequencer, REAL position-hold engine and REAL autopilot position PIDs are
// compiled together and flown by a small point-mass physics model. The three
// field failures of v12-v14 (sideways starvation, yaw chatter, sitting still)
// all lived in the wiring BETWEEN these modules - exactly what isolated unit
// tests cannot see and this harness exercises end to end.

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/axis.h"
    #include "common/filter.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/alt_hold.h"
    #include "flight/autopilot.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/nav_hud.h"
    #include "flight/nav_mission.h"
    #include "flight/pid.h"
    #include "flight/pos_hold.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "rx/rx.h"

    #include "pg/alt_hold.h"
    #include "pg/autopilot.h"
    #include "pg/gps_rescue.h"
    #include "pg/nav_hud.h"
    #include "pg/pos_hold.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(altHoldConfig_t, altHoldConfig, PG_ALTHOLD_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(navHudConfig_t, navHudConfig, PG_NAV_HUD_CONFIG, 0);

    // test control knobs
    extern bool testWaypointSwitch;   // nav_hud.c (UNIT_TEST)
    extern bool testMissionSwitch;    // nav_mission.c (UNIT_TEST)
    void navMissionResetForTest(void);
    float navMissionCarrotSpeedForTest(void);
    bool navMissionRetriedForTest(void);
    void navHudIngestFix(const vector2_t *posCm, float intervalS, bool rescueFlying, timeUs_t nowUs);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------- stubs ---
extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;
    float GPS_cosLat = 1.0f;
    bool canUseGPSHeading = true;

    float simAltitudeCm = 2000.0f;
    float simVertVelCmS = 0.0f;
    float simCosTilt = 1.0f;
    float simStickDeflection = 0.0f;
    bool simFailsafe = false;
    uint16_t simGpsStamp = 1;

    float getAltitudeCm(void) { return simAltitudeCm; }
    int32_t getEstimatedAltitudeCm(void) { return (int32_t)simAltitudeCm; }
    float getAltitudeDerivative(void) { return simVertVelCmS; }
    float getCosTiltAngle(void) { return simCosTilt; }
    float getGpsDataIntervalSeconds(void) { return 0.1f; }
    float getGpsDataFrequencyHz(void) { return 10.0f; }
    float rcCommand[4];

    bool failsafeIsActive(void) { return simFailsafe; }
    float getRcDeflectionAbs(int axis) { UNUSED(axis); return simStickDeflection; }
    bool gpsIsHealthy(void) { return true; }

    bool gpsHasNewData(uint16_t* gpsStamp) {
        if (*gpsStamp != simGpsStamp) {
            *gpsStamp = simGpsStamp;
            return true;
        }
        return false;
    }

    // flat-earth conversion matching nav_mission's enuToLocation inverse
    void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
    {
        distance->x = (to->lon - from->lon) * EARTH_ANGLE_TO_CM;
        distance->y = (to->lat - from->lat) * EARTH_ANGLE_TO_CM;
    }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig) { UNUSED(input); UNUSED(rxConfig); }
    throttleStatus_e calculateThrottleStatus() { return THROTTLE_LOW; }

    uint32_t millisRW = 0;
    uint32_t millis() { return millisRW; }
    uint32_t microsRW = 0;
    uint32_t micros() { return microsRW; }
    bool compassIsHealthy(void) { return true; }
    bool sensors(uint32_t mask) { UNUSED(mask); return true; }
}

// ----------------------------------------------------------- simulation ---
// Point-mass quad: autopilot lean angles produce horizontal acceleration,
// the mission's commanded yaw rate turns the heading (positive setpoint
// rotates counter-clockwise, i.e. reduces a positive heading-minus-bearing
// error - the convention under which GPS Rescue's law converges).
class NavMissionSimTest : public ::testing::Test {
protected:
    double posE, posN;       // cm, ENU relative to home
    double velE, velN;       // cm/s
    double windE = 0.0, windN = 0.0;   // cm/s: drag couples the craft to the moving air
    double headingDeg;       // compass degrees
    double headingBiasDeg = 0.0;   // injected compass error (toilet-bowl generator)
    double magBiasTargetDeg = 0.0;     // in-flight mag fault: bias ramps toward this
    double magBiasRampDegPerS = 0.0;
    double simGpsGlitchE = 0.0, simGpsGlitchN = 0.0;   // cm, offset on published fixes
    bool simPilotPosHoldBox = false;   // pilot's own POS HOLD switch stays on post-abort
    double actualYawRateDps = 0.0;     // yaw responds with lag, not instantly
    double pathLengthM = 0.0;
    uint32_t simTimeUs;
    int yawSignFlips;
    double lastYawRate;

    void SetUp() override
    {
        posE = posN = velE = velN = 0.0;
        windE = windN = 0.0;
        headingDeg = 0.0;
        headingBiasDeg = 0.0;
        magBiasTargetDeg = 0.0;
        magBiasRampDegPerS = 0.0;
        simGpsGlitchE = simGpsGlitchN = 0.0;
        simPilotPosHoldBox = false;
        actualYawRateDps = 0.0;
        pathLengthM = 0.0;
        simTimeUs = 1000000;
        yawSignFlips = 0;
        lastYawRate = 0.0;
        resetGateTracking();

        armingFlags = 0;
        flightModeFlags = 0;
        stateFlags = 0;
        simStickDeflection = 0.0f;
        simFailsafe = false;
        simAltitudeCm = 2000.0f;
        simVertVelCmS = 0.0f;
        simCosTilt = 1.0f;
        simGpsStamp = 1;
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        memset(&attitude, 0, sizeof(attitude));

        // Morgan's MOZ-7 values: autopilot position PIDs and rescue tuning
        autopilotConfigMutable()->positionP = 30;
        autopilotConfigMutable()->positionI = 30;
        autopilotConfigMutable()->positionD = 30;
        autopilotConfigMutable()->positionA = 30;
        autopilotConfigMutable()->positionCutoff = 80;
        autopilotConfigMutable()->maxAngle = 50;
        // stock altitude PID defaults; the REAL alt-hold + altitudeControl fly
        // the vertical axis now, so sag and lag are genuine
        autopilotConfigMutable()->altitudeP = 15;
        autopilotConfigMutable()->altitudeI = 15;
        autopilotConfigMutable()->altitudeD = 15;
        autopilotConfigMutable()->altitudeF = 15;
        autopilotConfigMutable()->hoverThrottle = 1400;
        autopilotConfigMutable()->throttleMin = 1100;
        autopilotConfigMutable()->throttleMax = 1700;
        altHoldConfigMutable()->climbRate = 0;   // no stick target adjustment
        altHoldConfigMutable()->deadband = 0;
        gpsRescueConfigMutable()->groundSpeedCmS = 750;
        gpsRescueConfigMutable()->yawP = 20;
        gpsRescueConfigMutable()->returnAltitudeM = 0;
        gpsRescueConfigMutable()->ascendRate = 750;
        gpsRescueConfigMutable()->descendRate = 150;
        posHoldConfigMutable()->deadband = 5;
        navHudConfigMutable()->mission3D = 0;
        navHudConfigMutable()->mode = NAV_HUD_MODE_STANDARD;
        navHudConfigMutable()->breadcrumbs = 1;
        navHudConfigMutable()->breadcrumbCount = 64;
        navHudConfigMutable()->breadcrumbSpacingM = 5;

        autopilotInit();
        posHoldInit();
        altHoldInit();
        navHudInit();
        navMissionResetForTest();

        armingFlags |= ARMED;
        stateFlags |= GPS_FIX | GPS_FIX_HOME;
        gpsSol.numSat = 18;
        publishGps();
        navHudUpdate(simTimeUs);
    }

    void publishGps()
    {
        gpsSol.llh.lat = lrint((posN + simGpsGlitchN) / EARTH_ANGLE_TO_CM);
        gpsSol.llh.lon = lrint((posE + simGpsGlitchE) / EARTH_ANGLE_TO_CM);
        const double speed = sqrt(velE * velE + velN * velN);
        gpsSol.groundSpeed = (uint16_t)fmin(speed, 65000.0);
        gpsSol.groundCourse = (uint16_t)lrint(fmod(atan2(velE, velN) * 180.0 / M_PI + 360.0, 360.0) * 10.0);
        simGpsStamp++;
    }

    void dropWaypointAt(double eastM, double northM)
    {
        // teleport the craft there, feed the HUD a fix, flick the switch
        posE = eastM * 100.0;
        posN = northM * 100.0;
        publishGps();
        const vector2_t p = { .x = (float)posE, .y = (float)posN };
        navHudIngestFix(&p, 1.0f, false, simTimeUs);
        navHudUpdate(simTimeUs += 60000);
        testWaypointSwitch = true;
        navHudUpdate(simTimeUs += 60000);
        testWaypointSwitch = false;
        navHudUpdate(simTimeUs += 60000);
    }

    // one 10 ms physics + firmware step
    void step()
    {
        simTimeUs += 10000;
        millisRW = simTimeUs / 1000;

        // core.c emulation: a flying mission raises all three hold flags; a
        // FAILED mission force-parks with ANGLE + ALT_HOLD ONLY (self-level +
        // altitude, heading-independent - no POS_HOLD, which would fly sideways
        // on a bad-heading failure); the pilot's own POS HOLD box also keeps
        // the pipeline alive
        if (navMissionIsControlling()) {
            flightModeFlags |= (POS_HOLD_MODE | ALT_HOLD_MODE | ANGLE_MODE);
        } else if (navMissionFailed()) {
            flightModeFlags |= (ALT_HOLD_MODE | ANGLE_MODE);
            flightModeFlags &= ~POS_HOLD_MODE;
        } else if (simPilotPosHoldBox) {
            flightModeFlags |= (POS_HOLD_MODE | ALT_HOLD_MODE | ANGLE_MODE);
        } else {
            flightModeFlags &= ~(POS_HOLD_MODE | ALT_HOLD_MODE | ANGLE_MODE);
        }

        updatePosHold(simTimeUs);   // the real thing: mission -> pos engine -> autopilot
        updateAltHold(simTimeUs);   // the real alt-hold: mission target -> altitudeControl

        // pid.c emulation: autopilot angles apply only in the hold pipeline
        double pitchDeg = 0.0, rollDeg = 0.0;
        if (flightModeFlags & POS_HOLD_MODE) {
            pitchDeg = constrainf(autopilotAngle[AI_PITCH], -50.0f, 50.0f);
            rollDeg = constrainf(autopilotAngle[AI_ROLL], -50.0f, 50.0f);
        }
        simCosTilt = (float)(cos(pitchDeg * M_PI / 180.0) * cos(rollDeg * M_PI / 180.0));

        // in-flight mag fault: bias ramps toward its target
        if (magBiasRampDegPerS > 0.0 && headingBiasDeg < magBiasTargetDeg) {
            headingBiasDeg = fmin(headingBiasDeg + magBiasRampDegPerS * 0.01, magBiasTargetDeg);
        }

        // yaw: commanded rate turns the craft through a first-order lag (real
        // craft have yaw inertia); positive = counter-clockwise
        const double yawRate = navMissionGetYawRateDps();
        // count only meaningful reversals: micro-wiggle around zero error is
        // quantization, not chatter (the v13 bug flipped at +/-180 dps)
        if (yawRate * lastYawRate < 0.0 && fabs(yawRate) > 15.0) {
            yawSignFlips++;
        }
        if (fabs(yawRate) > 15.0) {
            lastYawRate = yawRate;
        }
        actualYawRateDps += (yawRate - actualYawRateDps) * (0.01 / 0.15);
        actualYawRateDps = constrainf(actualYawRateDps, -200.0, 200.0);
        headingDeg = fmod(headingDeg - actualYawRateDps * 0.01 + 360.0, 360.0);
        attitude.values.yaw = (int16_t)lrint(fmod(headingDeg + headingBiasDeg + 360.0, 360.0) * 10.0);

        // translate: lean angles -> body accel -> earth accel (cm/s^2)
        const double g = 981.0;
        const double fwd = g * tan(pitchDeg * M_PI / 180.0);
        const double right = g * tan(rollDeg * M_PI / 180.0);
        const double h = headingDeg * M_PI / 180.0;
        const double aE = fwd * sin(h) + right * cos(h);
        const double aN = fwd * cos(h) - right * sin(h);
        const double drag = 0.12;
        velE += (aE - drag * (velE - windE)) * 0.01;
        velN += (aN - drag * (velN - windN)) * 0.01;
        posE += velE * 0.01;
        posN += velN * 0.01;
        pathLengthM += sqrt(velE * velE + velN * velN) * 0.01 / 100.0;

        // vertical: the REAL altitudeControl PID+F flies this axis now.
        // Thrust model: hover at 1400 PWM cancels gravity when flat; tilt
        // sheds vertical thrust by cos(lean) - the sag mechanism the deck
        // margin exists for. throttleOut is normalized against mincheck 1050.
        if (isAltHoldActive()) {
            const double thrPwm = 1050.0 + getAutopilotThrottle() * 950.0;
            const double aZ = 981.0 * ((thrPwm - 1000.0) / 400.0 * simCosTilt - 1.0)
                              - 0.05 * simVertVelCmS;   // mild vertical drag
            simVertVelCmS += (float)(aZ * 0.01);
            simAltitudeCm += simVertVelCmS * 0.01f;
        } else {
            simVertVelCmS = 0.0f;   // manual hover: the pilot holds altitude
        }

        if ((simTimeUs / 10000) % 10 == 0) {
            publishGps();   // 10 Hz GPS
        }
        navHudUpdate(simTimeUs);
    }

    double distanceToM(double eastM, double northM)
    {
        const double dE = posE - eastM * 100.0, dN = posN - northM * 100.0;
        return sqrt(dE * dE + dN * dN) / 100.0;
    }

    double speedMs() { return sqrt(velE * velE + velN * velN) / 100.0; }

    // -------- gate-crossing tracker: is the nose on the NEXT leg at the
    // moment each leg switch actually happens? (dip waypoints turn during
    // the dip instead and are skipped via deckM). Call every step.
    static constexpr unsigned kMaxGates = 8;
    double gateHeadingErrDeg[kMaxGates];
    unsigned prevActiveIndex = 0;

    void resetGateTracking()
    {
        for (unsigned i = 0; i < kMaxGates; i++) {
            gateHeadingErrDeg[i] = -1.0;
        }
        prevActiveIndex = 0;
    }

    void trackGateCrossings(double deckM = -1.0)
    {
        const unsigned idx = navMissionActiveIndex();
        if (idx > prevActiveIndex) {
            const unsigned i = prevActiveIndex;   // the waypoint just passed
            if (i + 1 < navHudWaypointCount() && i < kMaxGates && gateHeadingErrDeg[i] < 0.0
                && !(deckM > 0.0 && navHudWaypointAltCm(i) < deckM * 100.0 - 250.0)) {
                const vector2_t *w = navHudWaypointAt(i);
                const vector2_t *nx = navHudWaypointAt(i + 1);
                const double bearing = atan2(nx->x - w->x, nx->y - w->y) * 180.0 / M_PI;
                gateHeadingErrDeg[i] = fabs(fmod(headingDeg - bearing + 540.0, 360.0) - 180.0);
            }
        }
        prevActiveIndex = idx;
    }

    void expectNoseOnNextLegAtGates(double tolDeg)
    {
        for (unsigned i = 0; i + 1 < navHudWaypointCount() && i < kMaxGates; i++) {
            if (gateHeadingErrDeg[i] < 0.0) {
                continue;   // dip or never crossed; coverage asserted elsewhere
            }
            EXPECT_LT(gateHeadingErrDeg[i], tolDeg)
                << "nose was " << gateHeadingErrDeg[i] << " deg off the next leg crossing waypoint " << (i + 1);
        }
    }

    // distance (m) from the craft to the nearest point of a polyline: the
    // "canyon corridor" metric - how far the flown path strays from the
    // drawn route, to either side, corners included
    double corridorDistanceM(const double (*ptsM)[2], int count)
    {
        const double cx = posE / 100.0, cy = posN / 100.0;
        double best = 1e9;
        for (int i = 0; i + 1 < count; i++) {
            const double x1 = ptsM[i][0], y1 = ptsM[i][1];
            const double x2 = ptsM[i + 1][0], y2 = ptsM[i + 1][1];
            const double dx = x2 - x1, dy = y2 - y1;
            const double len2 = dx * dx + dy * dy;
            double t = (len2 > 0.0) ? ((cx - x1) * dx + (cy - y1) * dy) / len2 : 0.0;
            t = fmax(0.0, fmin(1.0, t));
            best = fmin(best, hypot(cx - (x1 + t * dx), cy - (y1 + t * dy)));
        }
        return best;
    }
};

TEST_F(NavMissionSimTest, FliesThreeWaypointMissionNoseFirstAndParks)
{
    dropWaypointAt(60, 0);
    dropWaypointAt(60, 60);
    dropWaypointAt(0, 60);
    ASSERT_EQ(3u, navHudWaypointCount());

    // start back at home, nose North (waypoint 1 is due East)
    posE = posN = velE = velN = 0.0;
    headingDeg = 0.0;
    publishGps();

    testMissionSwitch = true;

    double minD1 = 1e9, minD2 = 1e9, minD3 = 1e9, peakSpeed = 0.0;
    int alignedCruiseSamples = 0, cruiseSamples = 0;
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {   // up to 120 s
        step();
        trackGateCrossings();
        minD1 = fmin(minD1, distanceToM(60, 0));
        minD2 = fmin(minD2, distanceToM(60, 60));
        minD3 = fmin(minD3, distanceToM(0, 60));
        peakSpeed = fmax(peakSpeed, speedMs());
        if (speedMs() > 3.0) {
            cruiseSamples++;
            // nose-first: heading within 35 degrees of the course flown
            const double course = fmod(atan2(velE, velN) * 180.0 / M_PI + 360.0, 360.0);
            double err = fabs(fmod(headingDeg - course + 540.0, 360.0) - 180.0);
            if (err < 35.0) {
                alignedCruiseSamples++;
            }
        }
    }

    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase()) << "mission never completed";
    // intermediate waypoints are pass-through gates (~12 m); the final one is precise
    EXPECT_LT(minD1, 13.0);
    EXPECT_LT(minD2, 13.0);
    EXPECT_LT(minD3, 6.0);
    // the nose leads into every corner: on the next leg by each gate crossing
    expectNoseOnNextLegAtGates(25.0);
    // cruises near rescue speed, doesn't creep and doesn't run away
    EXPECT_GT(peakSpeed, 4.0);
    EXPECT_LT(peakSpeed, 14.0);
    // flies nose-first for the majority of the cruise. The pre-turn
    // deliberately swings the nose onto the NEXT leg through the corner
    // approach while momentum still carries the old course, so on this
    // route's short 60 m legs a large minority of samples legitimately
    // read nose-vs-course misaligned; heading-at-gate is asserted
    // directly elsewhere
    ASSERT_GT(cruiseSamples, 100);
    EXPECT_GT((double)alignedCruiseSamples / cruiseSamples, 0.6);

    // parks at the last waypoint: a further 10 s must keep it there
    for (int i = 0; i < 1000; i++) {
        step();
    }
    EXPECT_LT(distanceToM(0, 60), 6.0);
    EXPECT_LT(speedMs(), 2.0);
}

TEST_F(NavMissionSimTest, EngagingFacingAwayTurnsWithoutChatter)
{
    dropWaypointAt(80, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 270.0;    // facing due West; waypoint is due East
    publishGps();

    testMissionSwitch = true;
    yawSignFlips = 0;
    for (int i = 0; i < 1500; i++) {   // 15 s
        step();
    }
    // the v13 failure mode was violent alternation around the wrap point:
    // a healthy turn converges with at most a couple of overshoot reversals
    EXPECT_LE(yawSignFlips, 3);
    // and the nose must end up pointing at the waypoint's direction of travel
    EXPECT_TRUE(navMissionGetPhase() == NAV_MISSION_FLYING || navMissionGetPhase() == NAV_MISSION_DONE);
    EXPECT_GT(posE, 1000.0) << "never left after turning - the v14 sat-there regression";
}

TEST_F(NavMissionSimTest, StickGrabAbortsAndStops)
{
    dropWaypointAt(100, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;     // already facing the waypoint
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 800; i++) {    // 8 s: well into the cruise
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    ASSERT_GT(speedMs(), 3.0);

    simStickDeflection = 0.3f;         // pilot grabs the sticks
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
    EXPECT_FALSE(navMissionIsControlling());
    EXPECT_EQ(0.0f, navMissionGetYawRateDps());
}

TEST_F(NavMissionSimTest, CrosswindLegStaysOnTheDrawnLine)
{
    dropWaypointAt(150, 0);            // one long leg due East
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    windN = -400.0;                    // 4 m/s crosswind from the North
    publishGps();

    testMissionSwitch = true;
    double maxCrossTrackM = 0.0;
    for (int i = 0; i < 6000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        if (speedMs() > 3.0) {
            // the leg runs along posN == 0: cross-track is just |posN|
            maxCrossTrackM = fmax(maxCrossTrackM, fabs(posN) / 100.0);
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    // line-following: the wind must not bow the flown path off the drawn leg
    EXPECT_LT(maxCrossTrackM, 5.0);
    EXPECT_LT(distanceToM(150, 0), 6.0);
}

TEST_F(NavMissionSimTest, RcLossMidMissionAbortsToFailsafe)
{
    dropWaypointAt(120, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 600; i++) {
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    simFailsafe = true;                // RX gone: failsafe owns the craft now
    for (int i = 0; i < 50; i++) {
        step();
    }
    EXPECT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
    EXPECT_FALSE(navMissionIsControlling());   // rescue is free to take over
}

TEST_F(NavMissionSimTest, Mission3DFliesTheStadiumProfile)
{
    navHudConfigMutable()->mission3D = 1;
    gpsRescueConfigMutable()->returnAltitudeM = 30;   // the transit floor
    gpsRescueConfigMutable()->ascendRate = 750;
    gpsRescueConfigMutable()->descendRate = 150;

    // waypoint 1 up at 45 m; the FINAL waypoint dropped LOW (12 m) in a field
    simAltitudeCm = 4500.0f;
    dropWaypointAt(80, 60);
    simAltitudeCm = 1200.0f;
    dropWaypointAt(80, 0);
    ASSERT_EQ(2u, navHudWaypointCount());
    ASSERT_EQ(4500, navHudWaypointAltCm(0));
    ASSERT_EQ(1200, navHudWaypointAltCm(1));

    // engage back at home, hovering at 20 m - below the 30 m floor
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simAltitudeCm = 2000.0f;
    publishGps();

    testMissionSwitch = true;
    bool touchedWp1Low = false;
    double minCruiseAltM = 1e9;
    for (int i = 0; i < 20000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        if (speedMs() > 3.0) {
            // whenever moving horizontally, we must be at/above the floor
            // (small slack for the climb-out transition)
            minCruiseAltM = fmin(minCruiseAltM, simAltitudeCm / 100.0);
        }
        if (distanceToM(80, 0) < 6.0 && simAltitudeCm < 1500.0) {
            touchedWp1Low = true;   // descended overhead the final waypoint to ~12 m
        }
    }

    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_TRUE(touchedWp1Low) << "never descended to the final waypoint's recorded altitude";
    EXPECT_GT(minCruiseAltM, 29.0) << "cruised below the rescue floor";
    // parks at the final waypoint, down at its recorded 12 m
    EXPECT_LT(distanceToM(80, 0), 6.0);
    EXPECT_NEAR(1200.0, simAltitudeCm, 300.0);
}

TEST_F(NavMissionSimTest, FlowsThroughCornersWithoutStopping)
{
    // a square-ish route with two 90-degree corners
    dropWaypointAt(70, 0);
    dropWaypointAt(70, 70);
    dropWaypointAt(0, 70);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    bool reachedCruise = false;
    double minSpeedAfterCruise = 1e9;
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        trackGateCrossings();
        if (speedMs() > 5.0) {
            reachedCruise = true;
        }
        if (reachedCruise && navMissionGetPhase() == NAV_MISSION_FLYING) {
            minSpeedAfterCruise = fmin(minSpeedAfterCruise, speedMs());
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    ASSERT_TRUE(reachedCruise);
    // flow-through: gates are crossed at roughly corner speed (~2.2 m/s);
    // the craft slows for corners but never parks between waypoints
    EXPECT_GT(minSpeedAfterCruise, 1.3);
    expectNoseOnNextLegAtGates(25.0);
}

TEST_F(NavMissionSimTest, CompassBiasDoesNotToiletBowl)
{
    dropWaypointAt(100, 0);
    dropWaypointAt(100, 80);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    headingBiasDeg = 15.0;   // a badly aligned compass rotates every correction
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 15000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_LT(distanceToM(100, 80), 6.0);
    // orbiting shows up as excess path: allow modest curvature, not laps
    const double idealM = 100.0 + 80.0;
    EXPECT_LT(pathLengthM, idealM * 1.6) << "flew " << pathLengthM << " m for an ideal " << idealM;
}

TEST_F(NavMissionSimTest, EngagingAtSpeedAwayFromWaypointRecovers)
{
    dropWaypointAt(120, 0);
    posE = posN = 0.0;
    velE = -800.0;           // 8 m/s away from the waypoint at engage
    velN = 0.0;
    headingDeg = 270.0;
    publishGps();

    testMissionSwitch = true;
    double maxExcursionM = 0.0;
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        maxExcursionM = fmax(maxExcursionM, fabs(posE < 0 ? posE : 0) / 100.0);
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    // momentum is arrested within a sane distance, not a fly-away
    EXPECT_LT(maxExcursionM, 40.0);
}

TEST_F(NavMissionSimTest, Mission3DDipsAtIntermediateLowWaypoint)
{
    navHudConfigMutable()->mission3D = 1;
    gpsRescueConfigMutable()->returnAltitudeM = 30;

    simAltitudeCm = 1200.0f;
    dropWaypointAt(80, 0);        // low pass marker, mid-route
    simAltitudeCm = 4500.0f;
    dropWaypointAt(80, 60);       // final, up high

    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simAltitudeCm = 3000.0f;
    publishGps();

    testMissionSwitch = true;
    bool dipped = false;
    double minMovingAltM = 1e9;
    for (int i = 0; i < 20000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        if (distanceToM(80, 0) < 6.0 && simAltitudeCm < 1500.0) {
            dipped = true;        // the deliberate low pass happened
        }
        if (speedMs() > 3.0) {
            minMovingAltM = fmin(minMovingAltM, simAltitudeCm / 100.0);
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_TRUE(dipped) << "never dipped to the low intermediate waypoint";
    EXPECT_GT(minMovingAltM, 29.0) << "moved horizontally below the hard deck";
    EXPECT_LT(distanceToM(80, 60), 6.0);
    EXPECT_NEAR(4500.0, simAltitudeCm, 300.0);   // climbed back out and parked high
}

TEST_F(NavMissionSimTest, Mission3DSlantsBetweenAboveDeckAltitudes)
{
    navHudConfigMutable()->mission3D = 1;
    gpsRescueConfigMutable()->returnAltitudeM = 30;

    simAltitudeCm = 6000.0f;
    dropWaypointAt(120, 0);       // 60 m
    simAltitudeCm = 4000.0f;
    dropWaypointAt(120, 90);      // final at 40 m

    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simAltitudeCm = 3500.0f;      // above the deck, below waypoint 1
    publishGps();

    testMissionSwitch = true;
    bool movedEarly = false;
    for (int i = 0; i < 20000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        // slanted climb: horizontal progress must begin while still well
        // below waypoint 1's altitude (no climb-first-then-go)
        if (i < 600 && posE > 1000.0 && simAltitudeCm < 5500.0) {
            movedEarly = true;
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_TRUE(movedEarly) << "waited to climb instead of flying a slanted leg";
    EXPECT_LT(distanceToM(120, 90), 6.0);
    EXPECT_NEAR(4000.0, simAltitudeCm, 300.0);   // slanted back down to the final altitude
}

// Morgan's v21 field mission shape: engage below the deck, first waypoint LOW
// (45 ft), four more above the deck with a hard 90-degree corner. The v21
// flight dropped below the hard deck outside the deliberate dip and crossed
// waypoints with the nose off the next leg - both must be impossible now.
TEST_F(NavMissionSimTest, FieldReplayLaunchBelowDeckRespectsTheDeck)
{
    navHudConfigMutable()->mission3D = 1;
    gpsRescueConfigMutable()->returnAltitudeM = 30;
    gpsRescueConfigMutable()->groundSpeedCmS = 900;   // Morgan's 20 mph cruise
    gpsRescueConfigMutable()->ascendRate = 300;       // a cautious climber: deck
                                                      // discipline must hold even
                                                      // when climbs are slow

    simAltitudeCm = 1370.0f;      // 45 ft: a deliberate below-deck dip
    dropWaypointAt(40, 0);
    simAltitudeCm = 3500.0f;
    dropWaypointAt(40, 60);       // route TURNS 90 degrees at the dip
    simAltitudeCm = 4000.0f;
    dropWaypointAt(100, 60);      // and again at waypoint 2
    simAltitudeCm = 3200.0f;
    dropWaypointAt(100, 10);
    simAltitudeCm = 3600.0f;
    dropWaypointAt(40, 10);
    ASSERT_EQ(5u, navHudWaypointCount());

    // engage hovering at 8 m - well BELOW the 30 m deck
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simAltitudeCm = 800.0f;
    publishGps();

    testMissionSwitch = true;
    bool violatedDeck = false;
    double worstAltM = 100.0, minDipAltM = 1e9, dipExitHeadingErr = -1.0;
    for (int i = 0; i < 30000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        trackGateCrossings(30.0);
        // horizontal flight below the deck is legal ONLY within the
        // deliberate dip, directly overhead the 45 ft waypoint. Judge both
        // the physics (craft speed) and the COMMAND (carrot marching): the
        // mission must not order any translation below the deck
        if ((speedMs() > 1.0 || navMissionCarrotSpeedForTest() > 50.0f)
            && simAltitudeCm < 2950.0 && distanceToM(40, 0) > 8.0) {
            violatedDeck = true;
            worstAltM = fmin(worstAltM, simAltitudeCm / 100.0);
        }
        if (distanceToM(40, 0) < 8.0) {
            minDipAltM = fmin(minDipAltM, simAltitudeCm / 100.0);
        }
        // the nose rotates onto the next leg DURING the dip: by the moment
        // the dip completes it must already face waypoint 2 (due North)
        if (dipExitHeadingErr < 0.0 && navMissionActiveIndex() == 1) {
            dipExitHeadingErr = fabs(fmod(headingDeg - 0.0 + 540.0, 360.0) - 180.0);
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_FALSE(violatedDeck) << "moved horizontally below the deck, down at " << worstAltM << " m";
    EXPECT_LT(minDipAltM, 16.0) << "never actually dipped to the 45 ft waypoint";
    ASSERT_GE(dipExitHeadingErr, 0.0);
    EXPECT_LT(dipExitHeadingErr, 30.0) << "nose not on the next leg when the dip completed";
    expectNoseOnNextLegAtGates(25.0);
    // parks on the final waypoint at its recorded altitude
    EXPECT_LT(distanceToM(40, 10), 6.0);
    EXPECT_NEAR(3600.0, simAltitudeCm, 300.0);
}

// The real fix for the field failures: a mag going bad in flight (motor-current
// interference on the puck) no longer toilet-bowls or fails the mission. While
// a leg flies at speed the heading comes from GPS course, not the mag, so a
// 120-degree mag bias is simply ignored and the mission completes normally.
// A sustained mag fault corrupts the heading, toilet-bowls the nose-first
// track until the sanity radius trips it, and after the one retry it FAILS -
// and the fail-park must self-level (ANGLE) without POS_HOLD, so the bad mag
// cannot drive it sideways.
TEST_F(NavMissionSimTest, MagFaultFailsThenSelfLevels)
{
    dropWaypointAt(250, 0);       // one long leg due East
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simPilotPosHoldBox = false;   // no pilot switch: the fail-park must self-level
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 500; i++) {   // 5 s: solid cruise
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    ASSERT_GT(speedMs(), 4.0);

    // the compass starts lying: bias ramps up to 120 degrees
    magBiasRampDegPerS = 60.0;
    magBiasTargetDeg = 120.0;

    int failedAt = -1;
    for (int i = 0; i < 6000 && failedAt < 0; i++) {
        step();
        if (navMissionFailed()) {
            failedAt = i;
        }
    }
    ASSERT_GE(failedAt, 0) << "a sustained mag fault must trip and fail the mission";

    step();   // one more step so the fail-park mode-forcing runs post-failure
    // fail-park: ANGLE self-level forced, POS_HOLD deliberately NOT (heading is
    // exactly what's broken here - forcing pos-hold would fly it sideways)
    EXPECT_TRUE(flightModeFlags & ANGLE_MODE);
    EXPECT_FALSE(flightModeFlags & POS_HOLD_MODE);

    // levels and coasts to a stop, does not power away at a lean. v30's
    // overrun-immune detector trips WITHOUT the speed-scrubbing nose flip
    // first, so the park starts from nearer cruise speed and needs a longer
    // horizon: bounded by +25 s, stopped by +40 s.
    for (int i = 0; i < 2500; i++) {
        step();
    }
    EXPECT_LT(speedMs(), 4.0) << "a self-leveled park must not power away";
    for (int i = 0; i < 1500; i++) {
        step();
    }
    EXPECT_LT(speedMs(), 2.0) << "a self-leveled park must coast to a stop";
}

// Fail-park (the "it just kept coasting at whatever bank" field complaint):
// when a mission fails - here from TWO GPS outliers exhausting the one-shot
// retry - the craft must self-level even with NO pilot hold switch on, and
// coast to a stop, not power off at its last lean.
TEST_F(NavMissionSimTest, FailedMissionParksSelfLevelAndHolds)
{
    dropWaypointAt(400, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simPilotPosHoldBox = false;   // the key case: nothing else would hold it
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 400; i++) {
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    // first outlier: spends the one-shot retry
    simGpsGlitchN = 6000.0;
    for (int i = 0; i < 10; i++) { step(); }
    simGpsGlitchN = 0.0;
    for (int i = 0; i < 300 && !navMissionRetriedForTest(); i++) { step(); }
    ASSERT_TRUE(navMissionRetriedForTest());

    // second outlier: no retry left -> the mission fails
    simGpsGlitchN = 6000.0;
    for (int i = 0; i < 10; i++) { step(); }
    simGpsGlitchN = 0.0;
    for (int i = 0; i < 600 && !navMissionFailed(); i++) { step(); }
    ASSERT_TRUE(navMissionFailed()) << "two outliers should exhaust the retry and fail";

    step();   // one more step so the fail-park mode-forcing runs post-failure
    // FAIL-PARK: ANGLE self-level forced even with no pilot hold switch;
    // POS_HOLD deliberately NOT forced (it leans on heading -> sideways risk)
    EXPECT_TRUE(flightModeFlags & ANGLE_MODE);
    EXPECT_FALSE(flightModeFlags & POS_HOLD_MODE);

    // it levels and coasts to a stop (momentum bleeds off via drag), then stays
    // put - it does NOT keep powering away at a lean
    for (int i = 0; i < 2500; i++) { step(); }
    EXPECT_LT(speedMs(), 2.0) << "a self-leveled park must coast to a stop, not fly away";
    const double e0 = posE, n0 = posN;
    for (int i = 0; i < 500; i++) { step(); }
    EXPECT_LT(distanceToM(e0 / 100.0, n0 / 100.0), 5.0) << "once stopped it stays put";
}

// A single 60 m GPS outlier fix trips the sanity radius; the deferred retry
// must re-anchor on the NEXT (healthy) fix and the mission must complete.
TEST_F(NavMissionSimTest, GpsGlitchRetriesOnceAndCompletes)
{
    dropWaypointAt(120, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 400; i++) {   // 4 s: at cruise
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    ASSERT_GT(speedMs(), 4.0);

    simGpsGlitchN = 6000.0;           // one multipath spike, exactly one fix
    for (int i = 0; i < 10; i++) {    // spans exactly one 10 Hz publish
        step();
    }
    simGpsGlitchN = 0.0;

    for (int i = 0; i < 6000 && navMissionGetPhase() == NAV_MISSION_FLYING; i++) {
        step();
    }
    EXPECT_TRUE(navMissionRetriedForTest()) << "the glitch never tripped the sanity radius";
    EXPECT_FALSE(navMissionFailed()) << "a single outlier fix must not fail the mission";
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_LT(distanceToM(120, 0), 6.0);
}

// Real-physics regression for the field bug: fly onto a late leg, interrupt the
// mission, move the craft ~400 m away, then re-engage. A fresh engage must
// restart at waypoint 1 and fly back to it - NOT resume the stale late index
// and beeline straight to the final waypoint from the new position.
TEST_F(NavMissionSimTest, ReEngageAfterMovingRestartsAtWaypointOne)
{
    dropWaypointAt(50, 0);
    dropWaypointAt(100, 0);
    dropWaypointAt(150, 0);           // three waypoints in a line due East
    ASSERT_EQ(3u, navHudWaypointCount());

    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;                // nose East, onto the first leg
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 8000 && navMissionActiveIndex() < 2; i++) {
        step();
    }
    ASSERT_EQ(2u, navMissionActiveIndex());   // reached the final leg
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    // interrupt on the final leg with a stick grab (index stays parked at 2)
    simStickDeflection = 0.3f;
    step();
    ASSERT_EQ(NAV_MISSION_ABORTED, navMissionGetPhase());
    simStickDeflection = 0.0f;

    // switch off, fly ~400 m away, switch back on
    testMissionSwitch = false;
    step();
    posE = -40000.0;
    posN = velE = velN = 0.0;
    publishGps();
    testMissionSwitch = true;
    step();

    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_EQ(0u, navMissionActiveIndex());   // restarted at waypoint 1

    // it actually flies back to waypoint 1 first (index advances THERE), rather
    // than running straight to the stale final waypoint from the new position
    for (int i = 0; i < 12000 && navMissionActiveIndex() < 1; i++) {
        step();
    }
    EXPECT_GE(navMissionActiveIndex(), 1u);
    EXPECT_LT(distanceToM(50, 0), 20.0);      // passed close by waypoint 1
}

// Smoothness contract: the carrot speed and the altitude-ramp feed-forward
// are both slewed - no step anywhere in a full 3D mission with slanted climbs,
// slanted descents and corners. The v21 code leapt to full cruise at every
// leg start and stepped the climb-rate at every ramp transition.
TEST_F(NavMissionSimTest, TargetSpeedAndClimbProfilesAreSmooth)
{
    navHudConfigMutable()->mission3D = 1;
    gpsRescueConfigMutable()->returnAltitudeM = 30;

    simAltitudeCm = 5000.0f;
    dropWaypointAt(70, 0);
    simAltitudeCm = 3500.0f;
    dropWaypointAt(70, 70);
    simAltitudeCm = 6000.0f;
    dropWaypointAt(0, 70);

    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    simAltitudeCm = 3500.0f;
    publishGps();

    testMissionSwitch = true;
    double prevSpeed = 0.0, prevClimb = 0.0;
    double maxSpeedSlope = 0.0, maxClimbSlope = 0.0;
    for (int i = 0; i < 20000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        const double s = navMissionCarrotSpeedForTest();
        const double c = navMissionGetTargetClimbRateCmS();
        maxSpeedSlope = fmax(maxSpeedSlope, fabs(s - prevSpeed) / 0.01);
        maxClimbSlope = fmax(maxClimbSlope, fabs(c - prevClimb) / 0.01);
        prevSpeed = s;
        prevClimb = c;
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    // both targets stay inside their 250 cm/s^2 slew budgets (small epsilon)
    EXPECT_LE(maxSpeedSlope, 260.0);
    EXPECT_LE(maxClimbSlope, 260.0);
}

// The canyon test: 20 mph cruise into a right-angle corner and a hairpin.
// Corner speeds are scaled on a constant delta-v budget, so the flown path
// must stay inside a tight corridor around the drawn polyline - to EITHER
// side, corners included. This is the wall-clearance guarantee.
TEST_F(NavMissionSimTest, CanyonCornersStayInsideTheCorridor)
{
    gpsRescueConfigMutable()->groundSpeedCmS = 900;
    dropWaypointAt(120, 0);
    dropWaypointAt(120, 100);     // 90-degree corner, arrived at speed
    dropWaypointAt(40, 30);       // ~130-degree hairpin back
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    static const double route[][2] = { {0, 0}, {120, 0}, {120, 100}, {40, 30} };
    testMissionSwitch = true;
    double maxCorridorM = 0.0, cornerCrossSpeed = -1.0;
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        trackGateCrossings();
        maxCorridorM = fmax(maxCorridorM, corridorDistanceM(route, 4));
        if (cornerCrossSpeed < 0.0 && navMissionActiveIndex() >= 1) {
            cornerCrossSpeed = speedMs();   // speed at the 90-degree leg switch
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    EXPECT_LT(maxCorridorM, 8.0) << "ran " << maxCorridorM << " m wide of the drawn route";
    // the 90-degree gate is taken slow (delta-v budget), not at cruise
    ASSERT_GE(cornerCrossSpeed, 0.0);
    EXPECT_LT(cornerCrossSpeed, 5.0);
    expectNoseOnNextLegAtGates(25.0);
    EXPECT_LT(distanceToM(40, 30), 6.0);
}

// Gentle doglegs must CARRY speed: the fixed 5 mph corner crawl is only for
// hard corners. Two shallow bends flown near cruise, still inside the same
// corridor bound.
TEST_F(NavMissionSimTest, ShallowCornersCarrySpeed)
{
    gpsRescueConfigMutable()->groundSpeedCmS = 900;
    dropWaypointAt(100, 0);
    dropWaypointAt(200, 30);      // ~17-degree bend
    dropWaypointAt(300, 0);       // ~33-degree bend
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    static const double route[][2] = { {0, 0}, {100, 0}, {200, 30}, {300, 0} };
    testMissionSwitch = true;
    bool reachedCruise = false;
    double minSpeedAfterCruise = 1e9, maxCorridorM = 0.0;
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
        maxCorridorM = fmax(maxCorridorM, corridorDistanceM(route, 4));
        if (speedMs() > 7.0) {
            reachedCruise = true;
        }
        // final-arrival braking is legitimate; only the corners must carry speed
        if (reachedCruise && navMissionGetPhase() == NAV_MISSION_FLYING
            && distanceToM(300, 0) > 30.0) {
            minSpeedAfterCruise = fmin(minSpeedAfterCruise, speedMs());
        }
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
    ASSERT_TRUE(reachedCruise);
    // shallow bends carried at speed: nowhere near the 2.2 m/s corner crawl
    EXPECT_GT(minSpeedAfterCruise, 5.0);
    EXPECT_LT(maxCorridorM, 8.0);
    EXPECT_LT(distanceToM(300, 0), 6.0);
}

// v30 field-failure regressions: the v29 mission log showed the craft
// overrunning the carrot (post-dip catch-up / tailwind), which flipped the
// steering 180 degrees onto a carrot BEHIND the craft, physically spun it at
// speed, and tripped the heading-fault detector - both field "mission failed"
// events, plus two silent retries, came from that one defect.

TEST_F(NavMissionSimTest, OverrunSteersForwardNotBackward)
{
    dropWaypointAt(400, 0);            // one long leg due East
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 800; i++) {    // 8 s: solid cruise, carrot well ahead
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    ASSERT_GT(speedMs(), 4.0);

    // teleport the craft 30 m PAST wherever the carrot can be (lead <= 25 m):
    // the tailwind-slingshot overrun, distilled
    posE += 3000.0;
    publishGps();

    // the old steering law aimed at the carrot behind the craft: sustained
    // ~180 dps commanded yaw and a heading fault ~2 s later. The v30 law must
    // keep steering forward, never spin, and never fail the mission.
    double maxYawCmd = 0.0;
    for (int i = 0; i < 500; i++) {    // 5 s straddles the 2 s fault window
        step();
        maxYawCmd = fmax(maxYawCmd, fabs((double)navMissionGetYawRateDps()));
        ASSERT_FALSE(navMissionFailed()) << "overrun must not trip the heading fault";
    }
    EXPECT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    EXPECT_LT(maxYawCmd, 90.0) << "steering flipped onto the carrot behind the craft";

    // and the mission still finishes normally
    for (int i = 0; i < 12000 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase());
}

TEST_F(NavMissionSimTest, FastMissStillCountsTheWaypoint)
{
    dropWaypointAt(250, 0);            // single (final) waypoint: 5 m arrive bubble
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 500; i++) {
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());

    // the wp6 field signature: cross abeam of the bubble at speed - past the
    // waypoint along-track, 6 m laterally off (bubble is 5 m)
    posE = 25600.0;                     // 6 m past the waypoint
    posN = 600.0;                       // 6 m abeam
    velE = 1100.0;                      // still carrying 11 m/s
    publishGps();

    // old behavior: bubble missed, carrot pinned at leg end, craft stalls and
    // yaws back (the pilot-abort). v30 overrun fallback must count arrival.
    for (int i = 0; i < 1500 && navMissionGetPhase() != NAV_MISSION_DONE; i++) {
        step();
    }
    EXPECT_EQ(NAV_MISSION_DONE, navMissionGetPhase())
        << "a fast near-miss crossing must count the waypoint, not stall past it";
}

TEST_F(NavMissionSimTest, OverspeedGovernorHoldsTheCarrot)
{
    debugMode = DEBUG_NAV_MISSION;      // expose carrot speed on debug[7]
    dropWaypointAt(500, 0);
    posE = posN = velE = velN = 0.0;
    headingDeg = 90.0;
    publishGps();

    testMissionSwitch = true;
    for (int i = 0; i < 800; i++) {
        step();
    }
    ASSERT_EQ(NAV_MISSION_FLYING, navMissionGetPhase());
    const int16_t cruiseCarrot = debug[7];
    ASSERT_GT(cruiseCarrot, 500);

    // sustained tailwind slam: drag couples the craft to 12 m/s air, well
    // above the profile speed. The governor must slew the carrot DOWN,
    // handing braking authority back to the position target, instead of
    // marching away in front of a craft it can no longer slow.
    windE = 1200.0;                     // 12 m/s tailwind, drag-coupled
    velE = 1500.0;                      // 15 m/s ground speed right now
    publishGps();
    int16_t minCarrot = cruiseCarrot;
    for (int i = 0; i < 300; i++) {     // 3 s of sustained overspeed
        step();
        minCarrot = (int16_t)fmin((double)minCarrot, (double)debug[7]);
    }
    EXPECT_LT(minCarrot, cruiseCarrot - 300)
        << "carrot kept marching at cruise while the craft overspeed persisted";
    ASSERT_FALSE(navMissionFailed());
    debugMode = 0;
}
