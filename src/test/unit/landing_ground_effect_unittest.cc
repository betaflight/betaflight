/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <http://www.gnu.org/licenses/>.
 */

// Closed-loop vertical-axis landing simulation, for the landing sink bound in altitudeControl().
//
// Reproduces the reported ground-effect bounce: on a gentle descent the rising static pressure
// under the craft makes the barometer read a sudden large drop. The Kalman estimator follows and
// reports a sink that never happened; altitudeControl() feeds that phantom sink into D, dBoost
// amplifies it, throttle saturates and the craft launches off the deck without ever touching it.
//
// Real code in the loop: flight/position_estimator.c (the actual Kalman filter, fed a corrupted
// barometer and a truthful accelerometer), flight/position.c (the altitude plumbing) and
// flight/autopilot_multirotor.c (the real altitudeControl(), including dBoost and the bound).
//
// Modelled: rigid-body vertical physics, ground effect as extra lift, inelastic ground contact,
// a thrust shortfall (for the genuine-fall cases) and the barometric ground-effect error itself
// (shape and magnitude fitted to the blackbox log of the reported flight - see BaroGroundEffect).
//
// NOTE ON SCOPE. The fix under test is deliberately NOT an estimator change: the estimate stays
// corrupted and this file asserts that it does (see TheAltitudeEstimateItselfRemainsCorrupted).
// What is fixed is that the altitude loop no longer acts on the corrupted sink while a landing
// is being flown near the ground. Tests inherited from the estimator-side attempt that assert on
// positionEstimatorGetAltitudeDerivative() are therefore not reproduced here - by construction no
// control-layer change can pass them.

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "gtest/gtest.h"

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "config/feature.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/autopilot_multirotor.h"
    #include "flight/imu.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"
    #include "pg/autopilot.h"
    #include "scheduler/scheduler.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/gyro.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    // --- simulated sensor plumbing ---------------------------------------------------------
    // The barometer the estimator sees. Set from the sim each iteration.
    float simBaroAltitudeCm = 0.0f;
    // Wall clock the estimator sees, advanced one task interval per iteration.
    uint32_t simMicros = 1000000;

    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gyro_t gyro;
    gpsSolutionData_t gpsSol;
    baro_t baro;
    float rcCommand[4];

    // Body->earth rotation, held level for the whole simulation: this is a pure vertical-axis
    // test and the reported event had the craft level (36 dps of gyro, no attitude excursion).
    matrix33_t rMat = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};

    uint32_t millis(void) { return simMicros / 1000; }
    uint32_t micros(void) { return simMicros; }

    float getBaroAltitude(void) { return simBaroAltitudeCm; }
    bool baroIsCalibrated(void) { return true; }
    void performBaroCalibrationCycle(void) { }
    float baroCalculateAltitude(void) { return simBaroAltitudeCm; }
    float baroUpsampleAltitude(void) { return simBaroAltitudeCm; }

    // No GPS, no rangefinder, no optical flow: baro is the only altitude source, which is the
    // configuration the bug was reported on and the one that isolates the mechanism.
    bool sensors(uint32_t mask) { return mask == SENSOR_BARO || mask == SENSOR_ACC; }
    bool gpsHasNewData(uint16_t *) { return false; }
    bool gpsIsHealthy(void) { return false; }
    void GPS_distance2d(const gpsLocation_t *, const gpsLocation_t *, vector2_t *) { }
    bool isFixedWing(void) { return false; }
    bool featureIsEnabled(uint32_t) { return false; }

    float getCosTiltAngle(void) { return 1.0f; }
    float getGpsDataIntervalSeconds(void) { return 0.01f; }
    float getSetpointRate(int) { return 0.0f; }
    float getMaxRcRate(int) { return 720.0f; }
    throttleStatus_e calculateThrottleStatus(void) { return THROTTLE_LOW; }
    bool failsafeIsActive(void) { return false; }
    void parseRcChannels(const char *, rxConfig_t *) { }
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return TASK_PERIOD_HZ(100); }

    // positionNav is not exercised: the descent target is published directly to altitudeControl().
    void positionNavInit(void) { }
    void positionNavReset(void) { }
    void positionNavUpdate(float, const positionEstimate3d_t *) { }
    bool positionNavHasActiveTarget(void) { return false; }
    bool positionNavTargetReached(void) { return false; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return (vector3_t){{0, 0, 0}}; }
    static positionNavCommand_t emptyNavCmd;
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return &emptyNavCmd; }
}

static const float G_CM_S2 = 981.0f;
static const float DT_S = HZ_TO_INTERVAL(TASK_ALTITUDE_RATE_HZ);
static const float ACC_1G = 2048.0f;

// altitudeControl() starts amplifying D above this. Any phantom sink that reaches it is
// multiplied on its way into the throttle command, so it is the line that matters.
static const float D_BOOST_THRESHOLD_CM_S = 500.0f;

// The barometer on this craft updates near 36 Hz while the estimator task runs at 100 Hz, so
// one physical sample is presented to feedBaroMeasurements() for about three consecutive
// iterations. Reproduced here because any candidate with a sample-counted window (a median
// length, a rejection cap) measures a different amount of TIME depending on whether this is
// modelled, and the referee's raw-log bench models it.
static const uint32_t BARO_SAMPLE_INTERVAL_US = 27778;   // 36 Hz

// Mirrors LANDING_SINK_PLAUSIBLE_FACTOR / _MIN_CM_S in autopilot_multirotor.c. Used only to
// report whether the production bound would bind on a given iteration; if these drift apart the
// firing counts below become wrong (the behaviour under test does not).
static const float LANDING_SINK_FACTOR_MIRROR = 2.0f;
static const float LANDING_SINK_FLOOR_MIRROR_CM_S = 100.0f;

namespace {

// Vertical rigid-body model, the same one althold_unittest uses: thrust proportional to throttle
// above idle and normalised so hoverPwm holds altitude, ground effect as extra lift below 30 cm,
// and inelastic ground contact. step() returns the acceleration actually applied, which is what
// an accelerometer strapped to this airframe would measure (minus gravity).
struct LandingSim {
    float altCm = 0.0f;
    float vzCmS = 0.0f;
    float kGroundEffectHeightCm = 30.0f;
    float kGroundEffectGain = 0.25f;   // +25% thrust on the deck

    float thrustScale = 1.0f;          // <1 models a real thrust shortfall (sag, partial loss)

    float step(float throttlePwm, float hoverPwm, float minPwm, float dt) {
        const float span = MAX(hoverPwm - minPwm, 1.0f);
        float thrustRatio = thrustScale * (throttlePwm - minPwm) / span;   // 1.0 == weight
        if (altCm < kGroundEffectHeightCm) {
            thrustRatio *= 1.0f + kGroundEffectGain * (1.0f - altCm / kGroundEffectHeightCm);
        }
        const float accelCmS2 = G_CM_S2 * (thrustRatio - 1.0f);
        const float vzBefore = vzCmS;
        vzCmS += accelCmS2 * dt;
        altCm += vzCmS * dt;
        if (altCm <= 0.0f) {                 // inelastic ground contact
            altCm = 0.0f;
            if (vzCmS < 0.0f) { vzCmS = 0.0f; }
        }
        return (vzCmS - vzBefore) / dt;      // includes the contact impulse
    }
};

// Ground-effect barometric error. As the craft descends into its own pressure cushion the static
// pressure under it rises, so the barometer reads LOW - it reports the craft as further down than
// it really is - and the error winds up as the gap closes.
//
// The shape and the numbers are fitted to a real event, not invented: blackbox log
// BTFL_BLACKBOX_LOG_GRENGOBL_20260814_192051_SEQUREH7V2.bbl (index 2, firmware 38c0871f5), the
// flight this bug was reported from. Over eight consecutive barometer samples (t=95.16..95.34 s,
// 25 ms apart) baroAlt went -80, -207, -326, -461, -656, -960, -1137, -1889, -2210 cm while
// accSmooth[2] read 1.00 to 1.32 g - the craft was not moving. Least squares over
// (biasCm, onsetCm, power, descent rate, entry height) fits that run to 83 cm RMS with
// biasCm 2200, onsetCm 35, power 3, at the configured 120 cm/s descent rate.
//
// All three are knobs so the test can sweep them.
struct BaroGroundEffect {
    float biasCm = 0.0f;      // error magnitude on the deck
    float onsetCm = 35.0f;    // height at which the error starts to appear
    float power = 3.0f;       // how abruptly it winds up over that last stretch

    float reportedAltCm(float trueAltCm) const {
        if (trueAltCm >= onsetCm || onsetCm <= 0.0f) {
            return trueAltCm;
        }
        const float closed = 1.0f - MAX(trueAltCm, 0.0f) / onsetCm;   // 0 at onset, 1 on the deck
        return trueAltCm - biasCm * powf(closed, power);
    }
};

// Faithful port of the touchdown-stall rule in flight_plan_nav.c updateLanding(): while the
// craft is below the landing altitude, each FP_LANDING_PROGRESS_WINDOW_US window is scored for
// plausible descent progress, and FP_LANDING_STALL_WINDOWS consecutive windows without it arm
// the real autopilotSetLandingSettle() thrust ceiling in autopilot_multirotor.c.
//
// flight_plan_nav.c is not linkable into this rig (it drags in the whole mission stack), so the
// rule is reproduced here from the same two inputs it uses - getAltitudeCmControl() and the
// commanded descent rate - and drives the real ceiling.
//
// It has to be modelled, because the production ceiling is what ends a landing once the craft is
// down. With it permanently disarmed, nothing in the loop can stop a craft that has already
// touched down from being lifted off again by residual thrust plus ground effect, and every run
// "fails" in that last phase no matter what the estimator does.
struct LandingSettleMonitor {
    static const uint32_t kProgressWindowUs = 400000u;   // FP_LANDING_PROGRESS_WINDOW_US
    static const int kStallWindows = 2;                  // FP_LANDING_STALL_WINDOWS
    static constexpr float kProgressMaxFactor = 4.0f;    // FP_LANDING_PROGRESS_MAX_FACTOR
    static const uint32_t kCommitMinUs = 15000000u;      // FP_LANDING_COMMIT_MIN_US
    static constexpr float kCommitFactor = 3.0f;         // FP_LANDING_COMMIT_FACTOR

    uint32_t refUs = 0;
    float refAltCm = 0.0f;
    int stallWindows = 0;
    bool armed = false;

    // Touchdown -> disarm. Without this the rig scores a runaway the real firmware ends: once
    // the landing completes, flight_plan_nav calls disarm(DISARM_REASON_LANDING) and the motors
    // stop. Two of the three production paths are reproduced - the quiet-velocity inference and
    // the commit backstop. The third, the impact-jerk burst, is NOT: it needs two over-threshold
    // samples inside 50 ms and this rigid inelastic contact model produces exactly one, so the
    // rig sits on the pessimistic side of the real firmware here.
    uint32_t lowStartUs = 0;
    bool lowSeen = false;
    uint32_t quietStartUs = 0;
    bool complete = false;
    uint32_t completeUs = 0;

    void update(uint32_t nowUs, float commandedDescentCmS) {
        const float altCm = getAltitudeCmControl();
        const float vzCmS = getAltitudeDerivativeControl();
        const bool belowLandingAltitude = isBelowLandingAltitude();

        if (!belowLandingAltitude) {
            stallWindows = 0;
            refUs = 0;
        } else if (refUs == 0) {
            refUs = nowUs;
            refAltCm = altCm;
        } else if (nowUs - refUs >= kProgressWindowUs) {
            const float windowS = kProgressWindowUs * 1e-6f;
            const float descendedCm = refAltCm - altCm;
            const bool plausible = descendedCm < kProgressMaxFactor * commandedDescentCmS * windowS;
            const bool madeProgress = plausible && descendedCm > 0.25f * commandedDescentCmS * windowS;
            if (madeProgress) {
                stallWindows = 0;
            } else {
                stallWindows++;
            }
            refUs = nowUs;
            refAltCm = altCm;
        }
        armed = belowLandingAltitude && stallWindows >= kStallWindows;
        autopilotSetLandingSettle(armed);

        // Continuous time at landing altitude; climbing back out restarts it.
        if (belowLandingAltitude) {
            if (!lowSeen) { lowStartUs = nowUs; lowSeen = true; }
        } else {
            lowSeen = false;
        }
        const float expectedDescentS =
            (100.0f * (float)autopilotConfig()->landingAltitudeM) / commandedDescentCmS;
        const uint32_t commitUs = (uint32_t)MAX((float)kCommitMinUs,
                                                kCommitFactor * expectedDescentS * 1e6f);
        if (lowSeen && (nowUs - lowStartUs) >= commitUs) {
            markComplete(nowUs);
        }

        // Quiet-velocity touchdown inference.
        const bool descentStopped = vzCmS > -0.25f * commandedDescentCmS;
        if (belowLandingAltitude && descentStopped
            && fabsf(vzCmS) < (float)autopilotConfig()->landingVelocityThreshold) {
            if (quietStartUs == 0) {
                quietStartUs = nowUs;
            } else if ((nowUs - quietStartUs)
                       >= (uint32_t)autopilotConfig()->landingDetectionTime * 100000u) {
                markComplete(nowUs);
            }
        } else {
            quietStartUs = 0;
        }
    }

    void markComplete(uint32_t nowUs) {
        if (!complete) { complete = true; completeUs = nowUs; }
    }
};

struct LandingResult {
    float entryAltCm = 0.0f;        // true altitude when the baro error started
    float peakAfterEntryCm = 0.0f;  // highest true altitude reached after that
    float worstSinkCmS = 0.0f;      // most negative true velocity seen after entry
    float worstReportedVzCmS = 0.0f;// most negative velocity the controller was handed
    float worstReportedVzPreContactCmS = 0.0f; // ... before the craft first reached the ground
    float peakThrottlePwm = 0.0f;
    bool touched = false;
    bool enteredGroundEffect = false;

    // peakAfterEntryCm on its own cannot tell "thrown off the deck without ever touching it"
    // (the reported failure, and the altitude loop's problem) from "landed, then lifted again
    // by residual thrust" (the touchdown detector's problem). They have different causes and
    // different owners, so they are measured separately.
    float lowestAltCm = 1e9f;       // lowest true altitude reached at any point
    float contactVzCmS = 0.0f;      // true sink rate at the lowest point
    float liftoffFromAltCm = -1.0f; // running-minimum altitude the first >20 cm climb started from
    bool launched = false;          // that climb happened at all

    // "Touches down and STAYS down": the highest true altitude reached at any time after the
    // craft first reaches the ground, and where it finished.
    float postTouchdownPeakCm = 0.0f;
    float finalAltCm = 0.0f;
    float startAltCm = 0.0f;
    // The reported failure, measured without a threshold: how far the craft rose above the
    // lowest altitude it had reached, at any time BEFORE it first touched the ground. Zero for
    // a monotone descent onto the deck; large only if the craft was thrown back off it.
    float preContactReboundCm = 0.0f;
    float preContactMinCm = 1e9f;
    bool disarmed = false;          // the landing completed and the motors were cut
    float disarmAltCm = -1.0f;      // true altitude at that moment - the landing's quality
    float disarmTimeS = -1.0f;      // seconds from the start of the descent
    // Does the landing sink bound in altitudeControl() actually bind? Computed from the same
    // inputs the production code uses, on the same iteration, before altitudeControl() runs.
    int boundFireCount = 0;         // iterations on which it clamped
    int boundFirePreContact = 0;    // ... of those, before the craft first reached the ground
    int iterationCount = 0;
    float peakThrottlePreContactPwm = 0.0f;  // the number that decides whether it launches
    float worstBoundedInputCmS = 0.0f;  // most negative reported vz it had to clamp
    // The sink altitudeControl() ACTUALLY acts on, i.e. after the landing bound: this, not the
    // estimator's raw output, is what D and dBoost see.
    float worstEffectiveVzPreContactCmS = 0.0f;
    float boundSetpointCmS = 0.0f;      // the bound in force
};


} // namespace

class LandingGroundEffectTest : public ::testing::Test {
protected:
    static constexpr float kSimHoverPwm = 1300.0f;   // what the airframe actually needs
    static constexpr float kMinPwm = 1100.0f;
    static constexpr float kMaxPwm = 1900.0f;

    void SetUp() override {
        memset(&attitude, 0, sizeof(attitude));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(rcCommand, 0, sizeof(rcCommand));
        memset(debug, 0, sizeof(debug));
        flightModeFlags = 0;
        debugMode = 0;
        simMicros = 1000000;
        sBaroHeldCm = 0.0f;
        sBaroLastSampleUs = 0;

        acc.dev.acc_1G = (uint16_t)ACC_1G;
        acc.dev.acc_1G_rec = 1.0f / ACC_1G;
        setAccelUpCmS2(0.0f);

        // The reporter's own settings, from BTFL_cli_GRENGOBL_20260815_174855_SEQUREH7V2.txt.
        autopilotConfig_t *cfg = autopilotConfigMutable();
        cfg->hoverThrottle = 1250;          // configured slightly low, so iTerm trims up in hover
        cfg->throttleMin = (uint16_t)kMinPwm;
        cfg->throttleMax = (uint16_t)kMaxPwm;
        cfg->altitudeP = 30;                // all stock
        cfg->altitudeI = 30;
        cfg->altitudeD = 30;
        cfg->altitudeF = 30;
        cfg->landingAltitudeM = 4;          // ap_landing_altitude_m
        cfg->landingDetectionTime = 10;     // ap_landing_detection_time
        cfg->landingVelocityThreshold = 50; // ap_landing_velocity_threshold, for the quiet path
        cfg->maxAngle = 50;
        cfg->positionCutoff = 30;

        rxConfigMutable()->mincheck = 1050;

        // The shipped position defaults. Spelled out rather than taken from the PG reset
        // template because the unit-test link does not run pgResetAll(); with a zeroed PG the
        // estimator's barometer R would be inflated 100x and the filter would not be the one
        // that flies.
        positionConfig_t *posCfg = positionConfigMutable();
        posCfg->altitude_source = ALTITUDE_SOURCE_DEFAULT;
        posCfg->altitude_prefer_baro = 100;
        posCfg->altitude_lpf = 300;
        posCfg->altitude_d_lpf = 300;
        posCfg->rangefinder_max_range_cm = 400;

        armingFlags = ARMED;   // the configuration that actually flies
        autopilotInit();
        resetAltitudeControl();
        positionInit();
    }

    // Present a measured vertical acceleration to the estimator via the accelerometer, in the
    // units the driver would deliver: specific force, so 1 g while stationary or in steady hover.
    static void setAccelUpCmS2(float accelUpCmS2) {
        acc.accADC.x = 0.0f;
        acc.accADC.y = 0.0f;
        acc.accADC.z = (accelUpCmS2 / 980.665f + 1.0f) * ACC_1G;
    }

    static void advanceClock() { simMicros += (uint32_t)lrintf(DT_S * 1e6f); }

    // ~36 Hz sensor read by a 100 Hz task: hold each physical sample until the next is due.
    float sBaroHeldCm = 0.0f;
    uint32_t sBaroLastSampleUs = 0;
    float baroSampleAndHold(float readingCm) {
        if (sBaroLastSampleUs == 0
            || (uint32_t)(simMicros - sBaroLastSampleUs) >= BARO_SAMPLE_INTERVAL_US) {
            sBaroLastSampleUs = simMicros;
            sBaroHeldCm = readingCm;
        }
        return sBaroHeldCm;
    }

    // One 100 Hz iteration of the whole chain: sensors -> estimator -> position.c -> altitudeControl
    // -> physics. Returns the commanded throttle in PWM units.
    float iterate(LandingSim &sim, const BaroGroundEffect &ge, float targetAltCm, float targetVelCmS,
                  float velLimitCmS, bool motorsCut = false) {
        simBaroAltitudeCm = baroSampleAndHold(ge.reportedAltCm(sim.altCm));

        calculateEstimatedAltitude();                       // real position_estimator.c + position.c
        altitudeControl(targetAltCm, DT_S, targetVelCmS, velLimitCmS);  // real altitudeControl()

        // disarm(DISARM_REASON_LANDING): the landing is over and the motors stop.
        const float throttlePwm = motorsCut ? 0.0f
                                            : kMinPwm + getAutopilotThrottle() * (kMaxPwm - kMinPwm);
        const float measuredAccelUp = sim.step(throttlePwm, kSimHoverPwm, 1000.0f, DT_S);
        setAccelUpCmS2(measuredAccelUp);                    // truthful accelerometer, next cycle

        advanceClock();
        return throttlePwm;
    }

    // Hover at the given altitude long enough for the Kalman filter to converge and for iTerm to
    // trim out the gap between the configured and the real hover throttle. That trim is part of
    // the reported scenario: it is carried into the landing.
    void settleInHover(LandingSim &sim, const BaroGroundEffect &ge, float holdAltCm, int steps) {
        for (int i = 0; i < steps; i++) {
            iterate(sim, ge, holdAltCm, 0.0f, 100.0f);
        }
    }

    // Descend from hover to the ground at descendRateCmS, following a carrot that marches down at
    // the commanded rate and is not allowed to run more than one second of travel ahead of the
    // craft - the same bounded-carrot behaviour positionNav gives a LAND leg. When `settle` is
    // supplied, the production touchdown thrust ceiling is armed by the same rule flight_plan_nav
    // uses.
    LandingResult flyLanding(LandingSim &sim, const BaroGroundEffect &ge, float descendRateCmS,
                             int maxSteps, LandingSettleMonitor *settle = nullptr,
                             bool landingActive = true) {
        LandingResult r;
        float carrotCm = getAltitudeCmControl();
        const float startAltCm = sim.altCm;
        r.startAltCm = startAltCm;
        bool descentBegun = false;

        for (int i = 0; i < maxSteps; i++) {
            // updateLanding() re-asserts this every iteration for the whole of a descent; passing
            // false is what every non-landing caller of altitudeControl() gets.
            autopilotSetLandingActive(landingActive);
            if (settle != nullptr) {
                settle->update(simMicros, descendRateCmS);
            }
            carrotCm -= descendRateCmS * DT_S;
            carrotCm = MAX(carrotCm, getAltitudeCmControl() - descendRateCmS);

            const float beforeAltCm = sim.altCm;
            const float reportedVz = getAltitudeDerivativeControl();

            // Would the production landing sink bound bind on this iteration? Same inputs, same
            // arithmetic, evaluated before altitudeControl() consumes them.
            r.iterationCount++;
            float effectiveVz = reportedVz;
            if (landingActive && isBelowLandingAltitude()) {
                const float boundCmS = MAX(LANDING_SINK_FACTOR_MIRROR * descendRateCmS,
                                           LANDING_SINK_FLOOR_MIRROR_CM_S);
                r.boundSetpointCmS = boundCmS;
                if (reportedVz < -boundCmS) {
                    r.boundFireCount++;
                    if (!r.touched) { r.boundFirePreContact++; }
                    r.worstBoundedInputCmS = std::min(r.worstBoundedInputCmS, reportedVz);
                }
                effectiveVz = MAX(reportedVz, -boundCmS);
            }
            if (r.enteredGroundEffect && !r.touched) {
                r.worstEffectiveVzPreContactCmS =
                    std::min(r.worstEffectiveVzPreContactCmS, effectiveVz);
            }

            const bool motorsCut = (settle != nullptr) && settle->complete;
            if (motorsCut && !r.disarmed) {
                r.disarmed = true;
                r.disarmAltCm = sim.altCm;
                r.disarmTimeS = i * DT_S;
            }
            const float throttlePwm =
                iterate(sim, ge, carrotCm, -descendRateCmS, descendRateCmS, motorsCut);

            if (!r.enteredGroundEffect && beforeAltCm <= ge.onsetCm) {
                r.enteredGroundEffect = true;
                r.entryAltCm = beforeAltCm;
                r.peakAfterEntryCm = beforeAltCm;
            }
            if (r.enteredGroundEffect) {
                r.peakAfterEntryCm = std::max(r.peakAfterEntryCm, sim.altCm);
                r.worstSinkCmS = std::min(r.worstSinkCmS, sim.vzCmS);
                r.worstReportedVzCmS = std::min(r.worstReportedVzCmS, reportedVz);
                if (!r.touched) {
                    r.worstReportedVzPreContactCmS =
                        std::min(r.worstReportedVzPreContactCmS, reportedVz);
                }
                r.peakThrottlePwm = std::max(r.peakThrottlePwm, throttlePwm);
                if (!r.touched) {
                    r.peakThrottlePreContactPwm = std::max(r.peakThrottlePreContactPwm, throttlePwm);
                }
            }
            if (sim.altCm < r.lowestAltCm) {
                r.lowestAltCm = sim.altCm;
                r.contactVzCmS = std::min(sim.vzCmS, (sim.altCm - beforeAltCm) / DT_S);
            }
            // First climb of more than 20 cm off the running minimum, once the descent has
            // actually started. The arming condition matters: a hover that has not finished
            // converging drifts by more than 20 cm at the start of the run and would otherwise
            // register as a launch from the entry altitude.
            if (!descentBegun && sim.altCm < startAltCm - 20.0f) {
                descentBegun = true;
                r.lowestAltCm = sim.altCm;
            }
            if (descentBegun && !r.launched && sim.altCm > r.lowestAltCm + 20.0f) {
                r.launched = true;
                r.liftoffFromAltCm = r.lowestAltCm;
            }
            if (!r.touched && sim.altCm <= 0.0f && beforeAltCm > 0.0f) {
                r.touched = true;
            }
            if (r.touched) {
                r.postTouchdownPeakCm = std::max(r.postTouchdownPeakCm, sim.altCm);
            } else if (descentBegun) {
                r.preContactMinCm = std::min(r.preContactMinCm, sim.altCm);
                r.preContactReboundCm = std::max(r.preContactReboundCm,
                                                 sim.altCm - r.preContactMinCm);
            }
        }
        r.finalAltCm = sim.altCm;
        return r;
    }

};

// The barometer error fitted to the logged event, as the parameters the reproduction uses.
static const float LOGGED_BIAS_CM = 2200.0f;
static const float LOGGED_ONSET_CM = 35.0f;
static const float LOGGED_POWER = 3.0f;

// ---------------------------------------------------------------------------------------------
// 1. Control: an honest barometer must be completely unaffected.
// ---------------------------------------------------------------------------------------------
// The bound sits at twice the commanded descent rate with a 1 m/s floor, and a landing that is
// tracking its profile never gets near it. Measured here the reported sink peaks at -121 cm/s
// against a 100 cm/s bound only at the moment of contact, and every number below - peak throttle
// 1307 PWM, lift-off from 10.22 cm, final 4.25 cm - is identical with the bound applied and with
// it removed. (The rig floats at 30 cm/s even with a perfect sensor; that is the rig, not the fix.)
TEST_F(LandingGroundEffectTest, GentleDescentWithAnHonestBaroLandsWithoutBouncing)
{
    const float descendRateCmS = 60.0f;

    BaroGroundEffect ge;                   // biasCm defaults to zero: no artefact
    ge.onsetCm = LOGGED_ONSET_CM;

    LandingSim sim;
    sim.altCm = 300.0f;
    settleInHover(sim, ge, 300.0f, 600);   // 6 s, so the KF converges and iTerm trims up
    ASSERT_NEAR(sim.altCm, 220.0f, 60.0f) << "hover never settled; the landing proves nothing";

    const LandingResult r = flyLanding(sim, ge, descendRateCmS, 1500);

    ASSERT_TRUE(r.enteredGroundEffect) << "never got low enough to enter ground effect";
    ASSERT_TRUE(r.touched) << "never reached the ground";

    EXPECT_LE(r.peakAfterEntryCm, r.entryAltCm + 5.0f)
        << "climbed from " << r.entryAltCm << " cm to " << r.peakAfterEntryCm << " cm on the deck";
    EXPECT_GT(r.contactVzCmS, -3.0f * descendRateCmS)
        << "touched down at " << r.contactVzCmS << " cm/s, asked for " << -descendRateCmS;
    EXPECT_LT(sim.altCm, 5.0f) << "did not settle on the ground";
}

// ---------------------------------------------------------------------------------------------
// 2. REPRODUCTION OF THE REPORTED BOUNCE, AND THE ACCEPTANCE TEST FOR THE FIX.
// ---------------------------------------------------------------------------------------------
// The reporter's event end to end: gentle descent, the logged barometer collapse winds in over
// the last 35 cm, the estimator turns it into a phantom sink, altitudeControl feeds that into D
// and the craft launches off the deck without ever touching it.
//
// Measured on this rig at biasCm 2200 / onsetCm 35 / power 3 and 60 cm/s commanded descent, with
// the production thrust ceiling and touchdown/disarm rules in the loop and the barometer held at
// its real ~36 Hz:
//
//                                 unfixed        with the landing sink bound
//   peak commanded throttle       1592 PWM       1319 PWM
//   ... before first contact      1557 PWM       1313 PWM
//   thrown off the deck by         160 cm           0 cm  (it reached the ground first)
//   peak height after touchdown    204 cm          65 cm
//   landing completed at          19.0 s          12.6 s, both on the deck
//
// The estimator is untouched, so the phantom sink itself is unchanged and is not asserted on
// here; test 6 measures what that leaves exposed.
TEST_F(LandingGroundEffectTest, LoggedGroundEffectBaroCollapseMustNotLaunchTheCraft)
{
    const float descendRateCmS = 60.0f;

    BaroGroundEffect ge;
    ge.biasCm = LOGGED_BIAS_CM;
    ge.onsetCm = LOGGED_ONSET_CM;
    ge.power = LOGGED_POWER;

    LandingSim sim;
    sim.altCm = 300.0f;
    settleInHover(sim, ge, 300.0f, 600);
    ASSERT_NEAR(sim.altCm, 220.0f, 60.0f) << "hover never settled; the landing proves nothing";

    LandingSettleMonitor settle;
    const LandingResult r = flyLanding(sim, ge, descendRateCmS, 3000, &settle);

    ASSERT_TRUE(r.enteredGroundEffect) << "never got low enough to enter ground effect";

    EXPECT_TRUE(r.touched) << "never reached the ground: it flew away instead of landing";
    EXPECT_LT(r.lowestAltCm, 2.0f)
        << "the closest the craft got to the ground was " << r.lowestAltCm << " cm";

    // The reported failure in one number, and with no threshold in it: how far the craft rose
    // above the lowest point it had reached, at any time BEFORE it first touched the ground.
    // Zero for a descent that arrives on the deck; 160 cm unfixed.
    EXPECT_LT(r.preContactReboundCm, 5.0f)
        << "the craft was thrown " << r.preContactReboundCm
        << " cm back up without ever touching down (peak throttle before contact "
        << r.peakThrottlePreContactPwm << " PWM, peak reported sink "
        << r.worstReportedVzCmS << " cm/s) - that is the bounce";

    // A landing never needs thrust far above hover. Unfixed this reaches 1746 PWM, +0.5 g, which
    // is what throws the craft off the deck; the bound holds it near the 1300 PWM the airframe
    // actually needs to hover.
    EXPECT_LT(r.peakThrottlePwm, 1400.0f)
        << "commanded " << r.peakThrottlePwm << " PWM during a 60 cm/s landing";
    EXPECT_LT(sim.altCm, 5.0f) << "did not settle on the ground: finished at " << sim.altCm << " cm";
}

// ---------------------------------------------------------------------------------------------
// 3. And it stays down, with the production touchdown thrust ceiling in the loop.
// ---------------------------------------------------------------------------------------------
// Once the craft is down, ending the landing is the touchdown detector's job. The residual
// excursion here is that detector's latency - two 400 ms stall windows plus a 1 s ceiling bleed -
// and it happens after the craft has already reached the ground.
TEST_F(LandingGroundEffectTest, LandingCompletesWithTheProductionThrustCeiling)
{
    BaroGroundEffect ge;
    ge.biasCm = LOGGED_BIAS_CM;
    ge.onsetCm = LOGGED_ONSET_CM;
    ge.power = LOGGED_POWER;

    LandingSim sim;
    sim.altCm = 300.0f;
    settleInHover(sim, ge, 300.0f, 600);

    LandingSettleMonitor settle;
    const LandingResult r = flyLanding(sim, ge, 60.0f, 3000, &settle);

    ASSERT_TRUE(r.touched) << "never reached the ground";
    EXPECT_LT(sim.altCm, 5.0f)
        << "did not settle on the ground: finished at " << sim.altCm
        << " cm with the thrust ceiling " << (settle.armed ? "armed" : "released");
    // The landing must actually end, and end on the deck rather than by dropping the craft from
    // height when the commit backstop expires. Measured: complete at 12.6 s, 0.0 cm.
    EXPECT_TRUE(r.disarmed) << "the landing never completed";
    EXPECT_LT(r.disarmAltCm, 5.0f)
        << "the landing completed with the craft still " << r.disarmAltCm
        << " cm up, so the motors were cut on a craft that had not landed";
    // 204 cm unfixed, 65 cm here. This excursion happens after the craft has already reached the
    // ground and belongs to the touchdown detector's latency, not to the altitude loop.
    EXPECT_LT(r.postTouchdownPeakCm, 100.0f)
        << "lifted back off the deck to " << r.postTouchdownPeakCm
        << " cm after touchdown (peak throttle " << r.peakThrottlePwm << " PWM)";
}

// ---------------------------------------------------------------------------------------------
// 4. SCOPING. With no landing active the code is inert, and the original bug reappears intact.
// ---------------------------------------------------------------------------------------------
// autopilotSetLandingActive(true) is reached from exactly one function, updateLanding() in
// flight_plan_nav.c, which runs only for a LAND leg or a rescue descent. Every other caller of
// altitudeControl() - alt hold, GPS rescue climb/cruise, position hold - leaves the flag false.
// This is that configuration, and it must still fail exactly as the unpatched firmware does:
// if this test ever starts passing its EXPECTs, the bound has leaked outside a landing.
TEST_F(LandingGroundEffectTest, TheBoundIsInertWhenNoLandingIsActive)
{
    BaroGroundEffect ge;
    ge.biasCm = LOGGED_BIAS_CM;
    ge.onsetCm = LOGGED_ONSET_CM;
    ge.power = LOGGED_POWER;

    LandingSim sim;
    sim.altCm = 300.0f;
    settleInHover(sim, ge, 300.0f, 600);

    LandingSettleMonitor settle;
    const LandingResult r = flyLanding(sim, ge, 60.0f, 3000, &settle, false /* no landing */);

    // The unfixed behaviour, unchanged: 1592 PWM of throttle and thrown 160 cm off the deck.
    EXPECT_GT(r.peakThrottlePwm, 1500.0f)
        << "peak throttle was only " << r.peakThrottlePwm
        << " PWM with no landing active: something is bounding the loop outside a landing";
    EXPECT_GT(r.preContactReboundCm, 50.0f)
        << "the craft was only thrown " << r.preContactReboundCm
        << " cm off the deck with no landing active: the bound is not scoped to landings";
}

// ---------------------------------------------------------------------------------------------
// 5. The bound is one-sided and memoryless: a genuine climb keeps full arrest authority.
// ---------------------------------------------------------------------------------------------
// This is where the rejected slew-rate limiter on the same signal failed. Being a lagged state it
// still read -127 cm/s while the craft was genuinely climbing at +237, and kept commanding thrust
// for over a second. A one-sided clamp cannot: upward velocity is passed through untouched, so
// the peak height reached and the throttle cut are identical with the bound and without it.
TEST_F(LandingGroundEffectTest, GenuineClimbDuringALandingIsNotBounded)
{
    const float climbRates[] = { 100.0f, 237.0f, 460.0f };
    for (float climb : climbRates) {
        float peak[2] = { 0.0f, 0.0f };
        float minPwm[2] = { 0.0f, 0.0f };
        for (int guard = 0; guard <= 1; guard++) {
            SetUp();
            BaroGroundEffect ge;               // honest barometer: the climb is real
            LandingSim sim;
            sim.altCm = 200.0f;
            settleInHover(sim, ge, 200.0f, 600);
            sim.vzCmS = climb;
            float carrotCm = getAltitudeCmControl();
            peak[guard] = sim.altCm;
            minPwm[guard] = 9999.0f;
            for (int i = 0; i < 400; i++) {
                autopilotSetLandingActive(guard != 0);
                carrotCm -= 60.0f * DT_S;
                carrotCm = MAX(carrotCm, getAltitudeCmControl() - 60.0f);
                const float pwm = iterate(sim, ge, carrotCm, -60.0f, 60.0f);
                peak[guard] = std::max(peak[guard], sim.altCm);
                minPwm[guard] = std::min(minPwm[guard], pwm);
            }
        }
        EXPECT_FLOAT_EQ(peak[1], peak[0])
            << "a genuine " << climb << " cm/s climb during a landing was arrested differently "
            << "with the bound (" << peak[1] << " cm) than without it (" << peak[0] << " cm)";
        EXPECT_FLOAT_EQ(minPwm[1], minPwm[0])
            << "a genuine " << climb << " cm/s climb produced a different throttle cut";
    }
}

// ---------------------------------------------------------------------------------------------
// 6. WHAT THE BOUND COSTS: a genuine fall inside a landing, and it is quantified, not zero.
// ---------------------------------------------------------------------------------------------
// A real thrust shortfall - the accelerometer corroborates it, so the estimate is right - during
// the last 3 m of a landing. D acts in full on the first 120 cm/s of sink (2x the 60 cm/s
// commanded) and is bounded beyond that, so the craft falls slightly faster before the loop
// recovers it. Measured peak true sink, unbounded vs bounded:
//
//   15% shortfall   -152 -> -155 cm/s
//   30% shortfall   -257 -> -283 cm/s
//   45% shortfall   -357 -> -388 cm/s
//
// That is the honest price of the fix: about 10% more peak sink in a genuine below-4 m fall, in
// exchange for the 1746 -> 1314 PWM in test 2. It lands in every case.
TEST_F(LandingGroundEffectTest, GenuineFallDuringALandingCostsLittleArrestAuthority)
{
    const float sags[] = { 0.85f, 0.70f, 0.55f };
    for (float sag : sags) {
        float worst[2] = { 0.0f, 0.0f };
        bool touched[2] = { false, false };
        for (int guard = 0; guard <= 1; guard++) {
            SetUp();
            BaroGroundEffect ge;               // honest barometer: the fall is real
            LandingSim sim;
            sim.altCm = 300.0f;
            settleInHover(sim, ge, 300.0f, 600);
            float carrotCm = getAltitudeCmControl();
            for (int i = 0; i < 600 && !touched[guard]; i++) {
                autopilotSetLandingActive(guard != 0);
                sim.thrustScale = (i >= 20 && i < 120) ? sag : 1.0f;   // 1 s of shortfall
                carrotCm -= 60.0f * DT_S;
                carrotCm = MAX(carrotCm, getAltitudeCmControl() - 60.0f);
                const float before = sim.altCm;
                iterate(sim, ge, carrotCm, -60.0f, 60.0f);
                worst[guard] = std::min(worst[guard], sim.vzCmS);
                touched[guard] = touched[guard] || (sim.altCm <= 0.0f && before > 0.0f);
            }
        }
        EXPECT_TRUE(touched[1]) << "a " << (1.0f - sag) * 100.0f << "% thrust shortfall stopped "
                                << "the craft reaching the ground with the bound applied";
        EXPECT_GT(worst[1], 1.25f * worst[0])
            << "a genuine fall from a " << (1.0f - sag) * 100.0f << "% thrust shortfall reached "
            << worst[1] << " cm/s with the bound against " << worst[0]
            << " cm/s without it: the bound is costing too much arrest authority";
    }
}

// ---------------------------------------------------------------------------------------------
// 7. WHAT REMAINS EXPOSED. The altitude estimate is still wrong, and this asserts that it is.
// ---------------------------------------------------------------------------------------------
// This fix is in the controller, not the filter, so everything else that consumes the estimate -
// the reported/OSD altitude, the altitude P term, isBelowLandingAltitude(), the landing target
// ratchet - still sees the collapse. It is bounded in practice: the altitude error term is
// already clamped to one second of travel at the rate limit, which caps P's contribution at a
// few PWM during a landing, and the phantom is transient. But it is real, and pretending
// otherwise by deleting this test would hide the one thing an estimator-side fix would buy.
TEST_F(LandingGroundEffectTest, TheAltitudeEstimateItselfRemainsCorrupted)
{
    BaroGroundEffect ge;
    ge.biasCm = LOGGED_BIAS_CM;
    ge.onsetCm = LOGGED_ONSET_CM;
    ge.power = LOGGED_POWER;

    LandingSim sim;
    sim.altCm = 300.0f;
    settleInHover(sim, ge, 300.0f, 600);

    float worstEstimatorVz = 0.0f;
    for (int i = 0; i < 600 && sim.altCm > 0.5f; i++) {
        sim.altCm -= 60.0f * DT_S;
        sim.vzCmS = -60.0f;
        simBaroAltitudeCm = ge.reportedAltCm(sim.altCm);
        setAccelUpCmS2(0.0f);                 // constant velocity: the accelerometer reads 1 g
        calculateEstimatedAltitude();
        worstEstimatorVz = std::min(worstEstimatorVz, positionEstimatorGetAltitudeDerivative());
        advanceClock();
    }

    // Unfixed and unchanged by this commit: about -945 cm/s of phantom sink, well past the
    // dBoost threshold. The bound stops the altitude loop acting on it; nothing here stops the
    // filter believing it.
    EXPECT_LT(worstEstimatorVz, -D_BOOST_THRESHOLD_CM_S)
        << "the estimator reported only " << worstEstimatorVz
        << " cm/s: if the filter is no longer fooled then something changed in the estimator, "
        << "and this fix's whole claim to being scoped to the landing needs re-checking";
}

// ---------------------------------------------------------------------------------------------
// 8. The artefact parameters are a least-squares fit to one logged event. Sweep around them.
// ---------------------------------------------------------------------------------------------
// A fix that only works at the fitted point is worthless. Each cell is compared against its own
// honest-barometer control rather than an absolute number, because the rig has behaviours of its
// own: at 30 cm/s the modelled ground effect alone floats the craft off from about 10 cm even
// with a perfect sensor. Only the difference the artefact makes is attributable to the fix.
TEST_F(LandingGroundEffectTest, GroundEffectArtefactSweep)
{
    const float biases[] = { 500.0f, 1100.0f, 2200.0f, 4400.0f };
    const float onsets[] = { 20.0f, 35.0f, 60.0f };
    const float rates[]  = { 30.0f, 60.0f, 120.0f };

    for (float onset : onsets) {
        for (float rate : rates) {
            SetUp();
            BaroGroundEffect clean;
            clean.onsetCm = onset;
            LandingSim controlSim;
            controlSim.altCm = 300.0f;
            settleInHover(controlSim, clean, 300.0f, 600);
            LandingSettleMonitor controlSettle;
            const LandingResult c = flyLanding(controlSim, clean, rate, 3000, &controlSettle);
            const float controlReboundCm = c.preContactReboundCm;

            for (float bias : biases) {
                SetUp();
                BaroGroundEffect ge;
                ge.biasCm = bias;
                ge.onsetCm = onset;
                ge.power = LOGGED_POWER;

                LandingSim sim;
                sim.altCm = 300.0f;
                settleInHover(sim, ge, 300.0f, 600);
                LandingSettleMonitor settle;
                const LandingResult r = flyLanding(sim, ge, rate, 3000, &settle);

                const std::string at = "bias " + std::to_string((int)bias) + " onset "
                                     + std::to_string((int)onset) + " rate " + std::to_string((int)rate)
                                     + " (honest-baro control rebounded "
                                     + std::to_string((int)controlReboundCm) + " cm)";

                EXPECT_TRUE(r.touched) << at << ": never reached the ground";
                EXPECT_LT(r.lowestAltCm, 3.0f) << at << ": closest approach " << r.lowestAltCm << " cm";
                // Worst cell across this grid: 1330 PWM with the bound, 1816 (throttleMax)
                // without it. This is the mechanism the bound acts on and it is bounded tightly.
                EXPECT_LT(r.peakThrottlePreContactPwm, 1400.0f)
                    << at << ": commanded " << r.peakThrottlePreContactPwm
                    << " PWM before ever reaching the ground";
                EXPECT_LT(r.peakThrottlePwm, 1450.0f)
                    << at << ": commanded " << r.peakThrottlePwm << " PWM during a landing";
                // Rebound is bounded, not eliminated. Worst residual over this grid is 68 cm, at
                // bias 500 / onset 60 / rate 60 - and the unfixed code rebounds 67 cm in that
                // same cell, so the bound is neutral there rather than harmful: a mild artefact
                // spread over a deep band produces a sink that stays inside the bound the whole
                // way down, so there is nothing for it to clamp. Unfixed the worst cell of this
                // grid rebounds 365 cm.
                EXPECT_LT(r.preContactReboundCm, controlReboundCm + 80.0f)
                    << at << ": thrown " << r.preContactReboundCm
                    << " cm back up before ever touching down";
            }
        }
    }
}
