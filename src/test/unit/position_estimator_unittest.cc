#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "pg/pg_ids.h"

#include "common/maths.h"
#include "common/vector.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/position.h"
#include "flight/position_estimator.h"
#include "flight/position_filter.h"

// STATIC_UNIT_TESTED in position_estimator.c — gravity-removed earth-frame
// linear acceleration in ENU (cm/s^2).
void getLinearAccelENU(float *accelEast, float *accelNorth, float *accelUp);
bool gpsMeasurementReadyForFusion(timeUs_t nowUs);
float accelNoiseR(unsigned axis, float accel, float dt);

#include "io/gps.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/gyro.h"
#include "sensors/rangefinder.h"
#include "sensors/sensors.h"

PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
}

#include "gtest/gtest.h"

extern "C" {
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];

acc_t acc;
matrix33_t rMat;
gpsSolutionData_t gpsSol;

static uint32_t enabledSensors = 0;
static bool rfHealthy = false;
static float rfAltCm = 0.0f;
static bool rfUseFakeMicrosTimestamp = true;
static timeUs_t rfSampleTimeUs = 0;
static timeDelta_t rfSampleIntervalUs = 10000;
static float baroAltCm = 0.0f;
static bool baroUseFakeMicrosTimestamp = true;
static timeUs_t baroSampleTimeUs = 0;
static timeDelta_t baroSampleIntervalUs = 10000;
static timeUs_t fakeMicros = 0;
static bool gpsAlwaysHasNewData = true;
static bool gpsDataIsNew = false;
static float gpsDataFrequencyHz = 100.0f;

bool sensors(uint32_t mask) { return (enabledSensors & mask) != 0; }
bool rangefinderIsHealthy(void) { return rfHealthy; }
int32_t rangefinderGetLatestAltitude(void) { return lrintf(rfAltCm); }
timeUs_t rangefinderGetLatestSampleTimeUs(void) { return rfUseFakeMicrosTimestamp ? fakeMicros : rfSampleTimeUs; }
timeDelta_t rangefinderGetSampleIntervalUs(void) { return rfSampleIntervalUs; }
float getBaroAltitude(void) { return baroAltCm; }
timeUs_t getBaroLatestSampleTimeUs(void) { return baroUseFakeMicrosTimestamp ? fakeMicros : baroSampleTimeUs; }
timeDelta_t getBaroSampleIntervalUs(void) { return baroSampleIntervalUs; }

timeUs_t micros(void) { return fakeMicros; }

bool gpsHasNewData(uint16_t *gpsStamp)
{
    if (!gpsAlwaysHasNewData && !gpsDataIsNew) {
        return false;
    }

    gpsDataIsNew = false;
    (*gpsStamp)++;
    return true;
}

float getGpsDataFrequencyHz(void) { return gpsDataFrequencyHz; }

void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *dest)
{
    // Simplified (no Earth-scale projection): return integer-unit deltas so sign tests work.
    dest->x = (float)(to->lon - from->lon);
    dest->y = (float)(to->lat - from->lat);
}
}

static void stepEstimator(unsigned count = 1)
{
    for (unsigned i = 0; i < count; i++) {
        fakeMicros += 10000; // 100 Hz
        positionEstimatorUpdate();
    }
}

class PositionEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(&acc, 0, sizeof(acc));
        memset(&rMat, 0, sizeof(rMat));
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(debug, 0, sizeof(debug));

        // Identity rotation and 1G reciprocal scale so zero accel stays zero.
        rMat.m[NWU_N][X] = 1.0f;
        rMat.m[NWU_W][Y] = 1.0f;
        rMat.m[NWU_U][Z] = 1.0f;
        acc.dev.acc_1G_rec = 1.0f;
        acc.accADC.z = 1.0f;

        enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
        rfHealthy = true;
        rfAltCm = 100.0f;
        rfUseFakeMicrosTimestamp = true;
        rfSampleTimeUs = 0;
        rfSampleIntervalUs = 10000;
        baroAltCm = 100.0f;
        baroUseFakeMicrosTimestamp = true;
        baroSampleTimeUs = 0;
        baroSampleIntervalUs = 10000;
        gpsSol.llh.altCm = 100.0f;
        gpsSol.dop.pdop = 100; // pDOP 1.0

        stateFlags = GPS_FIX;
        armingFlags = ARMED;
        flightModeFlags = 0;
        fakeMicros = 0;
        gpsAlwaysHasNewData = true;
        gpsDataIsNew = false;
        gpsDataFrequencyHz = 100.0f;

        positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_PREFER;
        positionConfigMutable()->altitude_prefer_baro = 100;
        positionConfigMutable()->rangefinder_max_range_cm = 400;

        positionEstimatorInit();
    }
};

TEST_F(PositionEstimatorTest, GPSMeasurementsAreHeldAtAltitudeTaskRate)
{
    gpsAlwaysHasNewData = false;
    gpsDataFrequencyHz = 10.0f;
    gpsDataIsNew = true;
    positionEstimatorInit();

    EXPECT_TRUE(gpsMeasurementReadyForFusion(10000));

    // A 10 Hz GPS sample is reused for all ten 100 Hz altitude-task calls in
    // its 100 ms source interval.
    for (timeUs_t nowUs = 20000; nowUs <= 100000; nowUs += 10000) {
        EXPECT_TRUE(gpsMeasurementReadyForFusion(nowUs));
    }

    // Do not continue fusing a stale held sample if the next one is late.
    EXPECT_FALSE(gpsMeasurementReadyForFusion(110000));
}

TEST_F(PositionEstimatorTest, GPSUpsamplingTracksSourceFrequency)
{
    gpsAlwaysHasNewData = false;
    gpsDataFrequencyHz = 20.0f;
    gpsDataIsNew = true;
    positionEstimatorInit();

    EXPECT_TRUE(gpsMeasurementReadyForFusion(0));
    EXPECT_TRUE(gpsMeasurementReadyForFusion(40000));
    EXPECT_FALSE(gpsMeasurementReadyForFusion(50000));

    // The source rate can change at runtime as the receiver's nav interval changes.
    gpsDataFrequencyHz = 5.0f;
    gpsDataIsNew = true;
    EXPECT_TRUE(gpsMeasurementReadyForFusion(50000));
    EXPECT_TRUE(gpsMeasurementReadyForFusion(240000));
    EXPECT_FALSE(gpsMeasurementReadyForFusion(250000));
}

TEST_F(PositionEstimatorTest, RangefinderPreferFallsBackAndRecovers)
{
    // Establish offsets/baseline near zero.
    stepEstimator(10);
    const float baseline = positionEstimatorGetAltitudeCm();

    // Rangefinder unavailable: fallback should use baro/GPS updates.
    // Step long enough for the Z filter to settle on the new altitude. Its
    // response to a step is ~0.9 s to 63% and ~4 s to settle, so a short burst
    // would leave the estimate mid-transient and the comparison below would be
    // measuring convergence speed rather than the source preference.
    rfHealthy = false;
    baroAltCm = 150.0f;
    gpsSol.llh.altCm = 150.0f;
    stepEstimator(400); // 4 s at 100 Hz
    const float fallbackAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(fallbackAltitude, baseline + 10.0f);

    // Rangefinder becomes available again at a lower relative altitude.
    rfHealthy = true;
    rfAltCm = 120.0f; // +20cm relative to RF offset baseline
    stepEstimator(400);
    const float recoveredAltitude = positionEstimatorGetAltitudeCm();

    // In RANGEFINDER_PREFER, recovered altitude should be pulled down toward RF value.
    EXPECT_LT(recoveredAltitude, fallbackAltitude - 5.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: a rangefinder whose first valid sample arrives in mid-air defines
// the altitude frame, and it must define it from its own reading rather than inherit
// the estimate.
//
// RANGEFINDER_MIN_ALT_CM rejects samples below 10 cm, so a craft that climbs before
// the rangefinder comes into range gets its first accepted sample well above the
// origin - and until then the only Z source is the baro, which spikes metres on prop
// wash at throttle-up. A logged takeoff put the baro 4 m high inside 0.8 s and the
// seed landed in the middle of it, leaving the whole altitude frame 2.6 m out for the
// rest of the flight: OSD altitude, landing detection and rescue altitudes all wrong.
//
// So the rangefinder's AGL reading wins and the estimator is rebased onto it. The
// rebase has to be a frame shift, not a measurement: pushing a 2 m correction through
// kalmanUpdatePositionToVelocity() would differentiate into a phantom descent of over
// 10 m/s.
TEST_F(PositionEstimatorTest, RangefinderComingIntoRangeInFlightRebasesWithoutVelocitySpike)
{
    // No GPS, so nothing measures vertical velocity directly and the Z position
    // innovations are allowed to correct velocity - the path a bad rebase would spike.
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;

    // Rangefinder reports RANGEFINDER_OUT_OF_RANGE on the ground - which is what the
    // logged craft did, its sensor having a minimum range of its own - so no sample is
    // accepted and no offset is captured before takeoff.
    rfAltCm = -1.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(10);

    // The baro claims a 235 cm climb - the prop-wash excursion - and the Z filter
    // settles there because nothing else can contradict it yet.
    baroAltCm = 335.0f;
    stepEstimator(400); // 4 s at 100 Hz
    EXPECT_GT(positionEstimatorGetAltitudeCm(), 150.0f);

    // The rangefinder now comes into range for the first time and reports the truth:
    // 10 cm above the ground.
    rfAltCm = 10.0f;
    stepEstimator(1);

    // Rebased onto the rangefinder's frame, in one step, with no velocity spike.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 10.0f, 5.0f);
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 50.0f);

    // The baro's offset was shifted with the frame, so it does not drag the estimate
    // back over the following samples.
    stepEstimator(100);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 10.0f, 20.0f);
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 20.0f);
}

// The rebase must not fire under an altitude-holding mode: alt hold captured its
// target in the old frame and would chase a 2 m frame shift as a real altitude
// change. Alt hold is relative, so keeping the offset frame still holds the correct
// physical height - only the absolute-altitude consumers lose out, which is the safe
// trade. The seed then has to absorb the difference so there is still no spike.
TEST_F(PositionEstimatorTest, RangefinderComingIntoRangeUnderAltHoldDoesNotRebase)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;

    rfAltCm = -1.0f; // out of range on the ground, so no offset captured before takeoff
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(10);

    baroAltCm = 335.0f;
    stepEstimator(400);
    const float heldAltitude = positionEstimatorGetAltitudeCm();
    EXPECT_GT(heldAltitude, 150.0f);

    // Alt hold engaged before the rangefinder ever came into range.
    flightModeFlags = ALT_HOLD_MODE;

    rfAltCm = 10.0f;
    stepEstimator(1);

    // Frame left alone, so the altitude the mode is holding does not move...
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), heldAltitude, 5.0f);
    // ...and the seed absorbed the difference, so no phantom vertical velocity.
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 50.0f);

    stepEstimator(100);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), heldAltitude, 20.0f);
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 20.0f);
}

// The in-flight seed must not disturb the normal case: starting on the ground with
// the estimate near zero, the offset still reduces to the raw reading, so the
// rangefinder's relative altitude starts at zero and tracks the climb from there.
TEST_F(PositionEstimatorTest, RangefinderOffsetOnGroundIsTheRawReading)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;

    // In range from the start, sitting on the ground.
    rfAltCm = 15.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(50);

    // Offset == 15 cm, so relative rangefinder altitude is 0 and the estimate holds
    // near the origin rather than being pushed up by the raw 15 cm reading.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 10.0f);

    // Climbing 100 cm of rangefinder range moves the estimate by ~100 cm.
    rfAltCm = 115.0f;
    baroAltCm = 200.0f;
    stepEstimator(400);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 100.0f, 20.0f);
}

// The offset seed only covers the rangefinder's *first* valid sample. A rangefinder
// that loses lock and reacquires over different terrain steps its relative altitude
// with the offset already fixed, and with no other Z source the position covariance
// grows during the dropout - so the step arrives with a large gain and reaches
// velocity through kalmanUpdatePositionToVelocity(). This is the indoor
// rangefinder-only case: brief loss of lock, reacquired over a table.
TEST_F(PositionEstimatorTest, RangefinderStepAfterDropoutDoesNotSpikeVerticalVelocity)
{
    enabledSensors = SENSOR_RANGEFINDER; // nothing else to hold Z while the rangefinder is out
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_ONLY;

    rfAltCm = 15.0f;
    positionEstimatorInit();
    stepEstimator(200);
    const float settledAltitude = positionEstimatorGetAltitudeCm();

    // Lock lost for 1 s, so the position covariance grows on process noise alone.
    rfHealthy = false;
    stepEstimator(100);

    // Reacquired over terrain 3 m lower - a step no airframe could have flown.
    rfHealthy = true;
    rfAltCm = 315.0f;
    stepEstimator(1);

    // Gated: no phantom climb rate, and the estimate has not jumped.
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 50.0f);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), settledAltitude, 20.0f);
}

// The gate must not lock out permanently. If the rangefinder is persistently
// telling us something the filter disagrees with, the filter is what is wrong, so
// after Z_INNOVATION_GATE_MAX_REJECTS the sample is re-anchored through the
// position-only path - correcting altitude without a velocity spike.
TEST_F(PositionEstimatorTest, PersistentRangefinderDisagreementReAnchorsWithoutVelocitySpike)
{
    enabledSensors = SENSOR_RANGEFINDER; // rangefinder alone, nothing else to anchor Z
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_ONLY;

    rfAltCm = 15.0f;
    positionEstimatorInit();
    stepEstimator(200);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 10.0f);

    // A sustained 3 m disagreement. The first samples are rejected, so the estimate
    // holds; MAX_REJECTS is 10, so within ~15 samples it must re-anchor.
    rfAltCm = 315.0f;
    stepEstimator(3);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 30.0f); // still held off

    stepEstimator(300);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 300.0f, 30.0f); // re-anchored
    EXPECT_TRUE(positionEstimatorIsValidZ());

    // Re-anchoring goes through the position-only path, so it must not have produced
    // the phantom velocity the gate exists to prevent.
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 50.0f);
}

// A genuine fast climb must pass the gate. The estimator has to keep tracking real
// vertical motion; a gate tight enough to clip normal flight would be worse than
// the spike it prevents. This is the case that ruled out a chi-square gate: with
// the accelerometer reporting nothing, a 5 m/s climb reaches ~12 sigma and 10 m/s
// ~24 sigma, past the ~17 sigma of a 2 m position step.
TEST_F(PositionEstimatorTest, GenuineClimbIsNotRejectedByStepGate)
{
    enabledSensors = SENSOR_BARO;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_BARO_ONLY;

    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(200);

    // Climb at 5 m/s for 2 s, stepping the baro every sample as a real climb would.
    for (unsigned i = 0; i < 200; i++) {
        baroAltCm += 5.0f; // 500 cm/s at 100 Hz
        stepEstimator(1);
    }

    // Tracked the climb rather than rejecting it.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 1000.0f, 100.0f);
    EXPECT_GT(positionEstimatorGetVerticalVelocity(), 400.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test for the velocity ratchet: baro and rangefinder measure height
// against origins that drift relative to each other, so if only one of them can
// write velocity, the other repeatedly displaces the position state and only the
// pull-back reaches velocity. The correction is rectified into a velocity bias that
// grows without bound.
//
// Reproduces a logged indoor hover: no GPS, rangefinder steady, baro drifting up at
// 10 cm/s. With only the rangefinder on kalmanUpdatePositionToVelocity() the
// vertical velocity estimate ramped to -95 cm/s at -9.1 cm/s^2 - a claimed 4 m
// descent - while the craft did not move. That fed the altitude D term and was only
// masked by the alt-hold I term winding down to cancel it.
TEST_F(PositionEstimatorTest, DriftingBaroAgainstSteadyRangefinderDoesNotRatchetVerticalVelocity)
{
    // No GPS, so nothing measures vertical velocity directly: every Z position
    // source must take the same update form.
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50; // as logged: baro still carries real weight

    rfAltCm = 80.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(100);

    // 11 s of hover: the rangefinder holds, the baro creeps up 0.1 cm per 100 Hz
    // sample. The drift is smooth, so the step gate accepts every sample - this is
    // sensor disagreement, not a discontinuity.
    float worstVelocity = 0.0f;
    for (unsigned i = 0; i < 1100; i++) {
        baroAltCm += 0.1f; // 10 cm/s
        stepEstimator(1);
        worstVelocity = fminf(worstVelocity, positionEstimatorGetVerticalVelocity());
    }
    const float velocityAt11s = positionEstimatorGetVerticalVelocity();

    // A stationary craft must not be reported as descending at any point.
    EXPECT_GT(worstVelocity, -20.0f);

    // And the estimate must be bounded rather than ramping: another 11 s of the same
    // disagreement may not move it further.
    for (unsigned i = 0; i < 1100; i++) {
        baroAltCm += 0.1f;
        stepEstimator(1);
    }
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), velocityAt11s, 10.0f);
}

// Cross-calibration has to null baro drift, not merely lag it. The correction is a
// first-order lag toward the ideal offset, so a *drifting* source leaves a standing
// error of driftRate * tau - and baro drift is a ramp. At the original 100 s time
// constant a 10 cm/s drift meant a 10 m standing error, which is a position
// disagreement between the two Z sources and drags the estimate off the anchor in
// proportion to the baro's share of the fused weight.
//
// Fixing the velocity ratchet does not cover this: that cancels the velocity bias for
// any tau, leaving the position error untouched. So this is asserted on position.
TEST_F(PositionEstimatorTest, BaroDriftIsCrossCalibratedAwayRatherThanDraggingTheEstimate)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    // Steady rangefinder anchors the frame at zero.
    rfAltCm = 80.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(200);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 10.0f);

    // 30 s of 10 cm/s baro drift - 3 m of it - against a rangefinder that has not
    // moved. Long enough for the lag to reach steady state at the rangefinder tau.
    for (unsigned i = 0; i < 3000; i++) {
        baroAltCm += 0.1f;
        stepEstimator(1);
    }

    // The estimate must still sit on the rangefinder, not part-way to the baro.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 15.0f);
}

// Regression test for the near-ground failure, reproducing a logged descent to ~8 cm.
//
// Prop wash recirculating off the ground destabilises static pressure, and the baro
// stops measuring height: its relative altitude swung to -280 cm while the rangefinder
// sat steady at 8-11 cm. Two things then went wrong together. RANGEFINDER_MIN_ALT_CM
// discarded the perfectly good 8 cm readings, so the estimator had no rangefinder input
// at all during the worst baro excursion it will ever see; and nothing distrusted the
// baro, so the fused estimate followed it to -59 cm with -47 cm/s of phantom descent
// and a ~140-unit throttle kick - in the landing regime, where that means a bounce.
//
// The step gate cannot help: at 13.7 Hz its allowance is 251 cm per sample and the
// largest step in the excursion was 84 cm, so not one sample of the flight ever
// tripped it. The excursion is a fast ramp, not a discontinuity.
TEST_F(PositionEstimatorTest, GroundEffectBaroExcursionDoesNotDragTheEstimateDown)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    // Settled in a low hover at 8 cm AGL - below RANGEFINDER_MIN_ALT_CM, so these are
    // exactly the samples that used to be thrown away.
    rfAltCm = 8.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(300);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 12.0f);

    // The baro collapses 280 cm over 0.25 s, at the same per-sample rate as the log,
    // while the rangefinder holds steady at 8 cm.
    for (unsigned i = 0; i < 25; i++) {
        baroAltCm -= 11.2f; // 280 cm over 25 samples
        stepEstimator(1);
    }

    // The rangefinder still anchors Z, so the estimate barely moves and there is no
    // phantom descent for the altitude controller to chase.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 20.0f);
    EXPECT_GT(positionEstimatorGetVerticalVelocity(), -20.0f);

    // Nor while the excursion persists.
    stepEstimator(100);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 20.0f);
    EXPECT_GT(positionEstimatorGetVerticalVelocity(), -20.0f);
}

// Regression test: the ground-effect derate must survive a rangefinder dropout.
//
// The rangefinder is what tells us we are in ground effect, so keying the derate on a
// live sample switched the protection off at the moment it was most needed - losing the
// anchor and the knowledge together. Logged during a 140 ms dropout at 9 cm AGL: the baro
// sat at -193 cm, received full trust, and pulled the estimate to -29 cm. Rangefinders
// drop out near their minimum range, so this correlates with the very regime the derate
// exists to protect - every invalid run in that flight began below 40 cm AGL.
TEST_F(PositionEstimatorTest, GroundEffectDerateSurvivesRangefinderDropout)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    // Settled in a low hover at 9 cm AGL, as logged.
    rfAltCm = 9.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(300);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 12.0f);

    // The baro dives into ground effect while the rangefinder drops out entirely - the
    // combination that used to defeat the derate. 460 cm of dive, as logged on a touchdown.
    rfAltCm = -1.0f; // RANGEFINDER_OUT_OF_RANGE
    for (unsigned i = 0; i < 46; i++) {
        baroAltCm -= 10.0f; // 460 cm over 460 ms
        stepEstimator(1);
    }

    // The latched height keeps the baro distrusted, so the estimate holds. 460 ms is past
    // the 421 ms dropout that defeated the original single-fade form, and inside the hold,
    // so the derate must still be at full strength here rather than 84% faded.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 20.0f);
    EXPECT_GT(positionEstimatorGetVerticalVelocity(), -25.0f);

    // Still holding at the end of the hold window. The bound is loose because this is a
    // harsher case than any flight: the baro is pinned at -460 cm with no rangefinder at
    // all, so the growing covariance gives even a 100x-derated baro some pull, and a few
    // tens of cm is unavoidable. What matters is that it stays bounded instead of tracking
    // the baro down - the single-fade form reached -422 cm here against -26 cm now.
    stepEstimator(50); // ~1.0 s total, the full hold
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 40.0f);

    // The rangefinder returns and must find the estimate where it left it, not 50 cm low.
    rfAltCm = 9.0f;
    stepEstimator(50);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 20.0f);
    EXPECT_GT(positionEstimatorGetVerticalVelocity(), -25.0f);
}

// The latch must not become a permanent veto on the baro. A rangefinder that stops
// reporting for good leaves the baro as the only altitude source there is, so the derate
// has to fade out - a degraded baro altitude beats none, and with no rangefinder the
// ground-effect question is not answerable anyway.
TEST_F(PositionEstimatorTest, GroundEffectLatchExpiresOnSustainedRangefinderLoss)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    rfAltCm = 9.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(300);

    // Rangefinder lost permanently, then a genuine 100 cm climb on the baro alone.
    rfAltCm = -1.0f;
    stepEstimator(250); // 2.5 s, clear of the whole hold-plus-fade release
    baroAltCm = 200.0f;

    // Checked one second in, not after it has had all the time in the world. With no
    // other Z source the covariance grows until even a 100x-derated baro eventually
    // pulls the estimate over, so a long window converges either way and proves nothing:
    // a latch that never released still reached 100 cm given 6 s. What distinguishes them
    // is the rate - ~106 cm here against ~20 cm with the latch stuck on.
    stepEstimator(100);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 100.0f, 35.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// The ground-effect derate keys off the rangefinder, so it must not fire when the
// rangefinder is not a Z source: under altitude_source = BARO_ONLY, distrusting the baro
// near the ground would leave nothing driving Z at all.
TEST_F(PositionEstimatorTest, GroundEffectDerateDoesNotApplyWhenBaroIsTheOnlyZSource)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_BARO_ONLY;

    rfAltCm = 8.0f; // in ground effect, but explicitly not a Z source
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(300);

    // The baro still drives altitude normally: a 100 cm climb is tracked.
    baroAltCm = 200.0f;
    stepEstimator(400);
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 100.0f, 25.0f);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Regression test: GPS_distance2d must be called as (arm, current) so that a craft
// East of the arm position produces a positive kfEast (East) position estimate.
// The bug had the arguments reversed — (current, arm) — giving a negative East estimate.
TEST_F(PositionEstimatorTest, GPSPositionEastOfArmIsPositive)
{
    // Run a couple of steps so the arm location is captured from the initial gpsSol (lon=0).
    stepEstimator(2);

    // Place the craft East of the arm (positive longitude delta).
    gpsSol.llh.lon = 100000;

    // Let the Kalman filter converge toward the GPS position measurement.
    stepEstimator(30);

    // position is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->position.v[ENU_E], 0.0f);
}

// Build rMat from roll/pitch/yaw using the exact quaternion + rotation-matrix
// formulas from imu.c (imuComputeQuaternionFromRPY + imuComputeRotationMatrix),
// so the matrix carries the firmware's body->earth NWU (North-West-Up) convention.
// Yaw follows the attitude/compass convention: 0 = North, +90 = East increases the
// reported heading, so a craft heading East is built with yawDeg = -90.
static void setRMatFromEuler(float rollDeg, float pitchDeg, float yawDeg)
{
    const float hr = DEGREES_TO_RADIANS(rollDeg) * 0.5f;
    const float hp = DEGREES_TO_RADIANS(pitchDeg) * 0.5f;
    const float hy = DEGREES_TO_RADIANS(yawDeg) * 0.5f;

    const float cr = cosf(hr), sr = sinf(hr);
    const float cp = cosf(hp), sp = sinf(hp);
    const float cy = cosf(hy), sy = sinf(hy);

    const float q0 = cr * cp * cy + sr * sp * sy;
    const float q1 = sr * cp * cy - cr * sp * sy;
    const float q2 = cr * sp * cy + sr * cp * sy;
    const float q3 = cr * cp * sy - sr * sp * cy;

    const float xx = q1 * q1, yy = q2 * q2, zz = q3 * q3;
    const float xy = q1 * q2, xz = q1 * q3, yz = q2 * q3;
    const float wx = q0 * q1, wy = q0 * q2, wz = q0 * q3;

    rMat.m[NWU_N][X] = 1.0f - 2.0f * yy - 2.0f * zz;
    rMat.m[NWU_N][Y] = 2.0f * (xy - wz);
    rMat.m[NWU_N][Z] = 2.0f * (xz + wy);
    rMat.m[NWU_W][X] = 2.0f * (xy + wz);
    rMat.m[NWU_W][Y] = 1.0f - 2.0f * xx - 2.0f * zz;
    rMat.m[NWU_W][Z] = 2.0f * (yz - wx);
    rMat.m[NWU_U][X] = 2.0f * (xz - wy);
    rMat.m[NWU_U][Y] = 2.0f * (yz + wx);
    rMat.m[NWU_U][Z] = 1.0f - 2.0f * xx - 2.0f * yy;
}

// Regression test for the East-West / magnitude bug discussed in #15321 and #15339.
// getLinearAccelENU must use matrixVectorMul (body->earth NWU) with East = -West;
// the previously-used matrixTrnVectorMul applied the inverse rotation, giving the
// wrong magnitude and a spurious North term.
//
// Scenario: heading East, 30 deg right roll, 0.5 g of forward (East) thrust.
// Gravity projects onto body Y and Z because of the roll, so accBF = (0.5, 0.5, 0.866).
// In the correct NWU formulation the gravity cross-terms cancel exactly, leaving
// the full 0.5 g of thrust on East and nothing on North or Up.
TEST_F(PositionEstimatorTest, EastThrustProducesEastAccelENU)
{
    constexpr float GRAVITY_CMSS = 980.665f;

    // rMat produced via imuComputeRotationMatrix's convention for heading East + 30 deg roll.
    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    // Forward (East) thrust of 0.5 g; gravity projects onto body Y and Z due to the roll.
    acc.dev.acc_1G_rec = 1.0f;
    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    float accelEast, accelNorth, accelUp;
    getLinearAccelENU(&accelEast, &accelNorth, &accelUp);

    EXPECT_NEAR(accelEast,  0.5f * GRAVITY_CMSS, 0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelNorth, 0.0f,                0.01f * GRAVITY_CMSS);
    EXPECT_NEAR(accelUp,    0.0f,                0.01f * GRAVITY_CMSS);
}

// accelNoiseR reports the accelerometer's measurement noise variance, measured from
// the signal: content above ACCEL_NOISE_CUTOFF_HZ (2 Hz) is noise, slower content is
// real acceleration. Floored at the R_ACCEL_XY a quiet craft has always used.
static float settleAccelNoiseR(unsigned axis, float dt, float seconds, float (*signal)(int))
{
    float r = 0.0f;
    const int steps = (int)(seconds / dt);
    for (int i = 0; i < steps; i++) {
        r = accelNoiseR(axis, signal(i), dt);
    }
    return r;
}

static float quietSignal(int) { return 0.0f; }
static float slowSignal(int i) { return 200.0f * sinf(2.0f * M_PIf * 0.2f * i * 0.01f); }   // 0.2 Hz, 200 cm/s^2
static float noisySignal(int i) { return 200.0f * ((i % 2) ? 1.0f : -1.0f); }               // 50 Hz square, 200 cm/s^2

TEST_F(PositionEstimatorTest, AccelNoiseRFloorsAtTheQuietValue)
{
    // A craft with no noise at all keeps the historic R_ACCEL_XY of 1500.
    EXPECT_NEAR(settleAccelNoiseR(0, 0.01f, 5.0f, quietSignal), 1500.0f, 1.0f);
}

TEST_F(PositionEstimatorTest, AccelNoiseRIgnoresRealAcceleration)
{
    // 2 m/s^2 of genuine 0.2 Hz acceleration is signal, not noise: R stays at the floor.
    EXPECT_NEAR(settleAccelNoiseR(0, 0.01f, 20.0f, slowSignal), 1500.0f, 300.0f);
}

TEST_F(PositionEstimatorTest, AccelNoiseRTracksVibration)
{
    // 200 cm/s^2 of alternating-sample vibration is pure noise: variance 200^2 = 40000,
    // clamped to R_ACCEL_XY_MAX of 20000.
    const float r = settleAccelNoiseR(1, 0.01f, 10.0f, noisySignal);
    EXPECT_NEAR(r, 20000.0f, 1.0f);
}

TEST_F(PositionEstimatorTest, AccelNoiseRRejectsBadArguments)
{
    EXPECT_FLOAT_EQ(accelNoiseR(0, 500.0f, 0.0f), 1500.0f);   // dt of zero
    EXPECT_FLOAT_EQ(accelNoiseR(9, 500.0f, 0.01f), 1500.0f);  // axis out of range
}

// Integration counterpart: with the same attitude/thrust, the IMU prediction alone
// (GPS held at the origin) must drive a positive East velocity estimate.
TEST_F(PositionEstimatorTest, EastThrustProducesPositiveEastVelocity)
{
    // Run a couple of steps so XY fusion is established with the arm at origin.
    stepEstimator(2);

    setRMatFromEuler(30.0f, 0.0f, -90.0f);

    acc.accADC.x = 0.5f;
    acc.accADC.y = 0.5f;    // sin(30) * 1g
    acc.accADC.z = 0.866f;  // cos(30) * 1g

    // GPS position and velocity remain zero so any non-zero velocity estimate is
    // driven purely by the IMU prediction.
    stepEstimator(100);

    // velocity is ENU; index by ENU_E so "East" is explicit rather than ".x".
    EXPECT_GT(positionEstimatorGetEstimate()->velocity.v[ENU_E], 0.0f);
}


// Regression guard for the diagonal-only Kalman update. A measurement of one
// state must correct the others through the P cross-terms. This matters most for
// XY position: with optical flow (or GPS velocity) as the only measurement there
// is no position measurement at all, so the P[pos][vel] term is the sole path by
// which anything ever corrects position. Drop it and XY position degenerates
// into an open-loop integral of velocity that drifts without bound.
// Index order matches kalmanGetPosition/Velocity/Acceleration: 0=pos, 1=vel, 2=accel.
// positionEstimatorResetZ() runs on every arm and disarm. It clears each Z source's
// offset, which redefines the relative altitude that source reports, so the source's step
// gate has to be cleared with it - a reference measured in the old frame would gate the
// new one against a height that no longer means anything, and reject perfectly good
// samples until the gate's escape hatch fires ten rejections later.
TEST_F(PositionEstimatorTest, ResetZClearsTheStepGatesWithTheOffsets)
{
    // Baro alone, so nothing else can refresh the Z measurement timestamp and mask a
    // gated-out baro.
    enabledSensors = SENSOR_BARO;
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_BARO_ONLY;

    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(50);

    // Climb smoothly so every step stays inside the gate's allowance and the gate's
    // reference follows the relative altitude up to ~400 cm.
    for (unsigned i = 0; i < 8; i++) {
        baroAltCm += 50.0f;
        stepEstimator(20);
    }
    ASSERT_TRUE(positionEstimatorIsValidZ());

    // Arm or disarm. The offset is cleared, so the next sample re-seeds it and relative
    // altitude returns to zero - a ~400 cm jump in the gate's terms, caused entirely by
    // the frame moving rather than the craft.
    positionEstimatorResetZ();
    EXPECT_FALSE(positionEstimatorIsValidZ()); // reset clears the timestamp

    // With the gate cleared too, the very first sample in the new frame is accepted and Z
    // is valid again immediately. With a stale reference it is rejected instead, and stays
    // invalid until the escape hatch re-anchors.
    stepEstimator(1);
    EXPECT_TRUE(positionEstimatorIsValidZ());
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 5.0f);
}

// Z validity has to drop when nothing can measure it. Under RANGEFINDER_ONLY the baro is
// excluded outright, so climbing past rangefinder_max_range_cm leaves no Z source at all
// and the estimate must be declared invalid once MEASUREMENT_TIMEOUT_US expires - not
// quietly dead-reckoned on the accelerometer and still reported as good.
TEST_F(PositionEstimatorTest, RangefinderOnlyAboveMaxRangeLosesZValidity)
{
    enabledSensors = SENSOR_BARO | SENSOR_RANGEFINDER; // baro present but excluded by source
    stateFlags = 0;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_RANGEFINDER_ONLY;
    positionConfigMutable()->rangefinder_max_range_cm = 400;

    rfAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(100);
    ASSERT_TRUE(positionEstimatorIsValidZ());

    // Above max range the sample is refused, so no Z measurement arrives at all.
    rfAltCm = 500.0f;
    stepEstimator(150); // 1.5 s, still inside the 2 s timeout
    EXPECT_TRUE(positionEstimatorIsValidZ()) << "must not drop before the timeout";

    stepEstimator(100); // now past 2 s
    EXPECT_FALSE(positionEstimatorIsValidZ());

    // And it recovers as soon as the rangefinder is back in range.
    rfAltCm = 100.0f;
    stepEstimator(5);
    EXPECT_TRUE(positionEstimatorIsValidZ());
}

// Cross-calibration corrects a drifting source against whichever anchor is active, and the
// rate is per anchor: fast against a low-noise rangefinder, slow against metre-noisy GPS.
// When both anchor at once the fastest must win, otherwise a rangefinder in range would be
// held back to the GPS rate and the baro drift it is supposed to null would survive.
TEST_F(PositionEstimatorTest, CrossCalibrationFollowsTheFastestActiveAnchor)
{
    enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = GPS_FIX; // GPS anchors as well, at its own much slower rate
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    rfAltCm = 80.0f;
    baroAltCm = 100.0f;
    gpsSol.llh.altCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(200);
    ASSERT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 15.0f);

    // 20 s of 10 cm/s baro drift against a rangefinder and a GPS altitude that both hold
    // station. The rangefinder's 2 s time constant must set the correction rate.
    for (unsigned i = 0; i < 2000; i++) {
        baroAltCm += 0.1f;
        stepEstimator(1);
    }

    // Held on the anchors rather than dragged by 2 m of baro drift.
    EXPECT_NEAR(positionEstimatorGetAltitudeCm(), 0.0f, 20.0f);
}

// The other half of zPositionInnovationDrivesVelocity(): when GPS velned measures vertical
// velocity directly, the Z position sources must take the narrow position-only form, so a
// standing baro-versus-rangefinder disagreement cannot also be counted as velocity.
//
// The signal is deliberately amplified to be measurable. GPS velocity is weakened to its
// 10x R cap via sAcc, and the baro is drifted at 30 cm/s rather than the ~10 cm/s seen in
// flight, because a fully trusted 100 Hz velocity measurement otherwise pins the state
// whichever form is used and the test would pass for the wrong reason. Ablating the
// predicate to force the wide form moves this from 0.0 to 9.7 cm/s; at the flight-realistic
// 10 cm/s drift it was only 3.2, which is where the 4 cm/s bound comes from. If GPS
// velocity R is ever retuned, re-derive that bound rather than trusting it.
TEST_F(PositionEstimatorTest, GpsVerticalVelocityKeepsZPositionOutOfTheVelocityState)
{
    enabledSensors = SENSOR_GPS | SENSOR_BARO | SENSOR_RANGEFINDER;
    stateFlags = GPS_FIX;
    positionConfigMutable()->altitude_source = ALTITUDE_SOURCE_DEFAULT;
    positionConfigMutable()->altitude_prefer_baro = 50;

    gpsSol.acc.sAcc = 3000;   // weakest GPS velocity the R scaling allows
    gpsSol.velned.velD = 0;   // GPS says stationary, and it is
    rfAltCm = 80.0f;
    baroAltCm = 100.0f;
    gpsSol.llh.altCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(200);

    // 20 s of baro drift against a rangefinder and GPS altitude that both hold station.
    for (unsigned i = 0; i < 2000; i++) {
        baroAltCm += 0.3f;
        stepEstimator(1);
    }

    // GPS measures the vertical velocity, so it stays at what GPS reports. Nothing from the
    // position disagreement leaks into it.
    EXPECT_NEAR(positionEstimatorGetVerticalVelocity(), 0.0f, 4.0f);
}

// Regression test: a horizontal state that has gone unmeasured for longer than the timeout
// must be discarded, not carried forward and reconciled.
//
// Logged with optical flow as the only XY source: the craft climbed above
// rangefinder_max_range_cm, which took optical flow with it, and XY ran open-loop for 22 s
// while the pilot manoeuvred. Position and velocity are a double integral of accelerometer
// error with nothing to anchor them, so the velocity state reached several m/s of phantom
// motion. Reconciling on reacquisition does not help: flow measures velocity alone, and with
// Pvv growing only to the order of the flow R the first sample moves velocity roughly half
// way. The position controller's D term acts on that velocity directly, so the moment the
// rangefinder came back into range it commanded a maximum-lean brake against motion that was
// not happening, and the craft flew into the ground.
//
// Driven with GPS rather than optical flow because the unit-test build does not compile
// USE_OPTICALFLOW, and the staleness path is common to both. Note the source is starved by
// stopping delivery of fixes while STATE(GPS_FIX) stays set: clearing the fix would instead
// take positionEstimatorWantXYFusion() false and disable XY altogether, which resets
// everything through a different path. Keeping XY enabled while it goes unmeasured is exactly
// the in-flight case, where POS_HOLD_MODE holds XY enabled and only optical flow is lost.
TEST_F(PositionEstimatorTest, StaleHorizontalStateIsDiscardedRatherThanCarried)
{
    enabledSensors = SENSOR_GPS | SENSOR_BARO;
    stateFlags = GPS_FIX;
    gpsSol.llh.altCm = 100.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(100);
    ASSERT_TRUE(positionEstimatorIsValidXY());

    // Accelerate hard while XY is still measured, so the velocity state holds something
    // substantial. Body X maps to North under the fixture's identity rotation.
    acc.accADC.x = 0.30f; // ~0.3 g
    stepEstimator(100);

    // Fixes stop arriving. Nothing measures horizontal position or velocity from here, but
    // XY stays enabled, so without the discard the state would integrate accelerometer error
    // open-loop for as long as the outage lasts.
    gpsAlwaysHasNewData = false;
    gpsDataIsNew = false;
    stepEstimator(150); // 1.5 s, still inside the 2 s timeout: dead reckoning is fine here
    EXPECT_TRUE(positionEstimatorIsValidXY()) << "must not drop before the timeout";

    // Past the timeout, and still accelerating at 0.3 g.
    stepEstimator(500); // 5 s
    EXPECT_FALSE(positionEstimatorIsValidXY());
    const positionEstimate3d_t *est = positionEstimatorGetEstimate();
    EXPECT_NEAR(est->velocity.v[ENU_N], 0.0f, 1.0f) << "stale velocity must be discarded";
    EXPECT_NEAR(est->position.v[ENU_N], 0.0f, 1.0f) << "stale position must be discarded";

    // Fixes return. The state was discarded, so its covariance is fresh and the measurement
    // lands cleanly instead of leaving a residue for the position controller's D term.
    acc.accADC.x = 0.0f;
    gpsAlwaysHasNewData = true;
    stepEstimator(2);
    EXPECT_TRUE(positionEstimatorIsValidXY());
    EXPECT_NEAR(positionEstimatorGetEstimate()->velocity.v[ENU_N], 0.0f, 50.0f);
}

// The GPS origin is what GPS Rescue and waypoint nav measure home against, so losing sight of
// the ground must not redefine it. Only the filter state is discarded on a lapse; the origin
// is left alone, which is what separates this from positionEstimatorResetXY().
TEST_F(PositionEstimatorTest, StaleHorizontalStateDoesNotMoveTheGpsOrigin)
{
    enabledSensors = SENSOR_GPS | SENSOR_BARO;
    stateFlags = GPS_FIX;
    gpsSol.llh.lat = 0;
    gpsSol.llh.lon = 0;
    gpsSol.llh.altCm = 100.0f;
    baroAltCm = 100.0f;
    positionEstimatorInit();
    stepEstimator(100);

    // Move East of the captured origin, and confirm the estimator sees the displacement.
    gpsSol.llh.lon = 500;
    stepEstimator(400);
    const float displacedEast = positionEstimatorGetEstimate()->position.v[ENU_E];
    ASSERT_GT(displacedEast, 100.0f);

    // Starve the source long enough for the state to be discarded, then resume it in the same
    // place. Position must return to the same displacement: if the origin had been recaptured
    // during the outage, the craft would now believe it was back at home.
    gpsAlwaysHasNewData = false;
    gpsDataIsNew = false;
    stepEstimator(500);
    ASSERT_FALSE(positionEstimatorIsValidXY());

    gpsAlwaysHasNewData = true;
    stepEstimator(400);
    EXPECT_NEAR(positionEstimatorGetEstimate()->position.v[ENU_E], displacedEast, 60.0f)
        << "origin moved: home would have shifted to where the dropout happened";
}

TEST(PositionFilterTest, ShiftPositionMovesPositionAndNothingElse)
{
    positionKalman_t kf;
    kalmanInit(&kf, 100.0f, 25.0f, -5.0f, 1000.0f, 2000.0f, 3000.0f, 3000.0f);

    // Predict forward so the covariance has real off-diagonal structure to disturb.
    for (int i = 0; i < 10; i++) {
        kalmanPredict(&kf, 0.01f);
    }
    ASSERT_GT(kf.P[0][1], 0.0f) << "no position/velocity correlation to check against";

    const float position = kalmanGetPosition(&kf);
    const float velocity = kalmanGetVelocity(&kf);
    const float acceleration = kalmanGetAcceleration(&kf);
    const float positionVar = kalmanGetPositionVariance(&kf);
    const float velocityVar = kalmanGetVelocityVariance(&kf);
    const float crossTerm = kf.P[0][1];

    kalmanShiftPosition(&kf, -260.0f);

    // A change of origin. Velocity and acceleration are rates, so they are invariant, and
    // the covariance describes spreads rather than absolute values, so it does not move
    // either. Anything else here would mean the shift had been smuggled in as a
    // measurement, which is precisely what it exists to avoid.
    EXPECT_FLOAT_EQ(kalmanGetPosition(&kf), position - 260.0f);
    EXPECT_FLOAT_EQ(kalmanGetVelocity(&kf), velocity);
    EXPECT_FLOAT_EQ(kalmanGetAcceleration(&kf), acceleration);
    EXPECT_FLOAT_EQ(kalmanGetPositionVariance(&kf), positionVar);
    EXPECT_FLOAT_EQ(kalmanGetVelocityVariance(&kf), velocityVar);
    EXPECT_FLOAT_EQ(kf.P[0][1], crossTerm);
}

TEST(PositionFilterTest, VelocityMeasurementCorrectsPositionState)
{
    positionKalman_t kf;
    kalmanInit(&kf, 0.0f, 0.0f, 0.0f, 10000.0f, 10000.0f, 10000.0f, 2000.0f);

    // Predict forward so the process model builds the position/velocity
    // correlation that a real flight would have.
    for (int i = 0; i < 10; i++) {
        kalmanPredict(&kf, 0.01f);
    }
    ASSERT_GT(kf.P[0][1], 0.0f) << "no position/velocity correlation to exploit";

    const float positionBefore = kalmanGetPosition(&kf);
    const float positionVarBefore = kalmanGetPositionVariance(&kf);

    kalmanUpdateVelocityToPosition(&kf, 100.0f, 400.0f); // measured 100 cm/s

    EXPECT_GT(kalmanGetVelocity(&kf), 0.0f);
    // The cross-term: a positive velocity innovation must also pull position up,
    // and must reduce the position variance.
    EXPECT_GT(kalmanGetPosition(&kf), positionBefore);
    EXPECT_LT(kalmanGetPositionVariance(&kf), positionVarBefore);
}

