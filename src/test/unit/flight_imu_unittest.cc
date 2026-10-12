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
#include <stdbool.h>
#include <limits.h>
#include <cmath>

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

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/compass/compass.h"
    #include "drivers/sensor.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "fc/rc.h"

    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"

    #include "io/gps.h"

    #include "rx/rx.h"

    #include "pg/autopilot.h"
    #include "pg/autopilot_wing.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"

    void imuComputeRotationMatrix(void);
    void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw);
    void imuUpdateEulerAngles(void);
    void imuMahonyAHRSupdate(float dt,
                             float gx, float gy, float gz,
                             bool useAcc, float ax, float ay, float az,
                             float headingErrMag, float headingErrCog,
                             const float dcmKpGain);
    float imuCalcMagErr(void);
    float imuCalcCourseErr(float courseOverGround);
    void imuRemoveCentripetalAcc(vector3_t *accAdc, const vector3_t *gyroRadS, float speedMs);
    float imuWingTurnSpeedMs(timeUs_t nowUs, float dtS, const vector3_t *gyroRadS);
    extern quaternion_t q;
    extern matrix33_t rMat;
    extern bool attitudeIsEstablished;

    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);

    PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
        .enabledFeatures = 0
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const float sqrt2over2 = sqrtf(2) / 2.0f;

void quaternion_from_axis_angle(quaternion_t* q, float angle, float x, float y, float z) {
    vector3_t a = {{x, y, z}};
    vector3Normalize(&a, &a);
    q->w = cos(angle / 2);
    q->x = a.x * sin(angle / 2);
    q->y = a.y * sin(angle / 2);
    q->z = a.z * sin(angle / 2);
}

TEST(FlightImuTest, TestCalculateRotationMatrix)
{
    #define TOL 1e-6

    // No rotation
    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;

    imuComputeRotationMatrix();

    EXPECT_FLOAT_EQ(1.0f, rMat.m[NWU_N][X]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_N][Y]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_N][Z]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_W][X]);
    EXPECT_FLOAT_EQ(1.0f, rMat.m[NWU_W][Y]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_W][Z]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_U][X]);
    EXPECT_FLOAT_EQ(0.0f, rMat.m[NWU_U][Y]);
    EXPECT_FLOAT_EQ(1.0f, rMat.m[NWU_U][Z]);

    // 90 degrees around Z axis
    q.w = sqrt2over2;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = sqrt2over2;

    imuComputeRotationMatrix();

    EXPECT_NEAR(0.0f, rMat.m[NWU_N][X], TOL);
    EXPECT_NEAR(-1.0f, rMat.m[NWU_N][Y], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_N][Z], TOL);
    EXPECT_NEAR(1.0f, rMat.m[NWU_W][X], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_W][Y], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_W][Z], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_U][X], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_U][Y], TOL);
    EXPECT_NEAR(1.0f, rMat.m[NWU_U][Z], TOL);

    // 60 degrees around X axis
    q.w = 0.866f;
    q.x = 0.5f;
    q.y = 0.0f;
    q.z = 0.0f;

    imuComputeRotationMatrix();

    EXPECT_NEAR(1.0f, rMat.m[NWU_N][X], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_N][Y], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_N][Z], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_W][X], TOL);
    EXPECT_NEAR(0.5f, rMat.m[NWU_W][Y], TOL);
    EXPECT_NEAR(-0.866f, rMat.m[NWU_W][Z], TOL);
    EXPECT_NEAR(0.0f, rMat.m[NWU_U][X], TOL);
    EXPECT_NEAR(0.866f, rMat.m[NWU_U][Y], TOL);
    EXPECT_NEAR(0.5f, rMat.m[NWU_U][Z], TOL);
}

TEST(FlightImuTest, TestUpdateEulerAngles)
{
    // No rotation
    memset(&rMat, 0.0, sizeof(float) * 9);

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(0, attitude.values.yaw);

    // 45 degree yaw
    memset(&rMat, 0.0, sizeof(float) * 9);
    rMat.m[NWU_N][X] = sqrt2over2;
    rMat.m[NWU_N][Y] = sqrt2over2;
    rMat.m[NWU_W][X] = -sqrt2over2;
    rMat.m[NWU_W][Y] = sqrt2over2;

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(450, attitude.values.yaw);
}

TEST(FlightImuTest, TestComputeQuaternionFromRPY)
{
    const quaternion_t savedQ = q;
    const attitudeEulerAngles_t savedAttitude = attitude;

    q.w = 1.0f;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = 0.0f;
    imuComputeRotationMatrix();

    imuComputeQuaternionFromRPY(0, 0, 900);
    imuUpdateEulerAngles();

    EXPECT_NEAR(sqrt2over2, q.w, 1e-6f);
    EXPECT_NEAR(0.0f, q.x, 1e-6f);
    EXPECT_NEAR(0.0f, q.y, 1e-6f);
    EXPECT_NEAR(-sqrt2over2, q.z, 1e-6f);
    EXPECT_EQ(900, attitude.values.yaw);

    q = savedQ;
    attitude = savedAttitude;
    imuComputeRotationMatrix();
}

TEST(FlightImuTest, TestSmallAngle)
{
    const float r1 = 0.898;
    const float r2 = 0.438;

    // given
    imuConfigMutable()->small_angle = 25;
    imuConfigure(0, 0);
    attitudeIsEstablished = true;

    // and
    memset(&rMat, 0.0, sizeof(float) * 9);

    // when
    imuComputeRotationMatrix();

    // expect
    EXPECT_FALSE(isUpright());

    // given
    rMat.m[NWU_N][X] = r1;
    rMat.m[NWU_N][Z] = r2;
    rMat.m[NWU_U][X] = -r2;
    rMat.m[NWU_U][Z] = r1;

    // when
    imuComputeRotationMatrix();

    // expect
    EXPECT_FALSE(isUpright());

    // given
    memset(&rMat, 0.0, sizeof(float) * 9);

    // when
    imuComputeRotationMatrix();

    // expect
    EXPECT_FALSE(isUpright());
}

testing::AssertionResult DoubleNearWrapPredFormat(const char* expr1, const char* expr2,
                                                  const char* abs_error_expr, const char* wrap_expr, double val1,
                                                  double val2, double abs_error, double wrap) {
    const double diff = remainder(val1 - val2, wrap);
    if (fabs(diff) <= abs_error) return testing::AssertionSuccess();

    return testing::AssertionFailure()
        << "The difference between " << expr1 << " and " << expr2 << " is "
        << diff << " (wrapped to 0 .. " << wrap_expr << ")"
        << ", which exceeds " << abs_error_expr << ", where\n"
        << expr1 << " evaluates to " << val1 << ",\n"
        << expr2 << " evaluates to " << val2 << ", and\n"
        << abs_error_expr << " evaluates to " << abs_error << ".";
}

#define EXPECT_NEAR_DEG(val1, val2, abs_error)                \
    EXPECT_PRED_FORMAT4(DoubleNearWrapPredFormat, val1, val2, \
                        abs_error, 360.0)

#define EXPECT_NEAR_RAD(val1, val2, abs_error)                \
    EXPECT_PRED_FORMAT4(DoubleNearWrapPredFormat, val1, val2, \
                        abs_error, 2 * M_PI)



class MahonyFixture : public ::testing::Test {
protected:
    vector3_t gyro;
    bool useAcc;
    vector3_t acc;
    bool useMag;
    vector3_t magEF;
    float cogGain;
    float cogDeg;
    float dcmKp;
    float dt;
    void SetUp() override {
        vector3Zero(&gyro);
        useAcc = false;
        vector3Zero(&acc);
        cogGain = 0.0;   // no cog
        cogDeg  = 0.0;
        dcmKp = .25;     // default dcm_kp
        dt = 1e-2;       // 100Hz update

        imuConfigure(0, 0);
        // level, poiting north
        setOrientationAA(0, {{1,0,0}});        // identity
    }
    virtual void setOrientationAA(float angleDeg, vector3_t axis) {
        quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(angleDeg), axis.x, axis.y, axis.z);
        imuComputeRotationMatrix();
    }

    float wrap(float angle) {
        angle = fmod(angle, 360);
        if (angle < 0) angle += 360;
        return angle;
    }
    float angleDiffNorm(vector3_t *a, vector3_t* b, vector3_t weight = {{1,1,1}}) {
        vector3_t tmp;
        vector3Scale(&tmp, b, -1);
        vector3Add(&tmp, &tmp, a);
        for (int i = 0; i < 3; i++)
            tmp.v[i] *= weight.v[i];
        for (int i = 0; i < 3; i++)
            tmp.v[i] = std::remainder(tmp.v[i], 360.0);
        return vector3Norm(&tmp);
    }
    // run Mahony for some time
    // return time it took to get within 1deg from target
    float imuIntegrate(float runTime, vector3_t * target) {
        float alignTime = -1;
        for (float t = 0; t < runTime; t += dt) {
            //     if (fmod(t, 1) < dt) printf("MagBF=%.2f %.2f %.2f\n", magBF.x, magBF.y, magBF.z);
            float headingErrMag = 0;
            if (useMag) {   // not implemented yet
                headingErrMag = imuCalcMagErr();
            }
            float headingErrCog = 0;
            if (cogGain > 0) {
                headingErrCog = imuCalcCourseErr(DEGREES_TO_RADIANS(cogDeg)) * cogGain;
            }

            imuMahonyAHRSupdate(dt,
                                gyro.x, gyro.y, gyro.z,
                                useAcc, acc.x, acc.y, acc.z,
                                headingErrMag, headingErrCog,
                                dcmKp);
            imuUpdateEulerAngles();
            // if (fmod(t, 1) < dt) printf("%3.1fs - %3.1f %3.1f %3.1f\n", t, attitude.values.roll / 10.0f, attitude.values.pitch / 10.0f, attitude.values.yaw / 10.0f);
            // remember how long it took
            if (alignTime < 0) {
                vector3_t rpy = {{attitude.values.roll / 10.0f, attitude.values.pitch / 10.0f, attitude.values.yaw / 10.0f}};
                float error = angleDiffNorm(&rpy, target);
                if (error < 1)
                    alignTime = t;
            }
        }
        return alignTime;
    }
};

class YawTest: public MahonyFixture, public testing::WithParamInterface<float> {
};

TEST_P(YawTest, TestCogAlign)
{
    cogGain = 1.0;
    cogDeg = GetParam();
    const float rollDeg = 30;    // 30deg pitch forward
    setOrientationAA(rollDeg, {{0, 1, 0}});
    vector3_t expect = {{0, rollDeg, wrap(cogDeg)}};
    // integrate IMU. about 25s is enough in worst case
    float alignTime = imuIntegrate(80, &expect);

    imuUpdateEulerAngles();
    // quad stays level
    EXPECT_NEAR_DEG(attitude.values.roll / 10.0, expect.x, .1);
    EXPECT_NEAR_DEG(attitude.values.pitch / 10.0, expect.y, .1);
    // yaw is close to CoG direction
    EXPECT_NEAR_DEG(attitude.values.yaw / 10.0, expect.z, 1);  // error < 1 deg
    if (alignTime >= 0) {
        printf("[          ] Aligned to %.f deg in %.2fs\n", cogDeg, alignTime);
    }
}

TEST_P(YawTest, TestMagAlign)
{
    float initialAngle = GetParam();

    // level, rotate to param heading
    quaternion_from_axis_angle(&q, -DEGREES_TO_RADIANS(initialAngle), 0, 0, 1);
    imuComputeRotationMatrix();

    vector3_t expect = {{0, 0, 0}};    // expect zero yaw

    vector3_t magBF = {{1, 0, .5}};    // use arbitrary Z component, point north

    mag.magADC = magBF;

    useMag = true;
    // integrate IMU. about 25s is enough in worst case
    float alignTime = imuIntegrate(30, &expect);

    imuUpdateEulerAngles();
    // quad stays level
    EXPECT_NEAR_DEG(attitude.values.roll / 10.0, expect.x, .1);
    EXPECT_NEAR_DEG(attitude.values.pitch / 10.0, expect.y, .1);
    // yaw is close to north (0 deg)
    EXPECT_NEAR_DEG(attitude.values.yaw / 10.0, expect.z, 1.0);  // error < 1 deg
    if (alignTime >= 0) {
        printf("[          ] Aligned from %.f deg in %.2fs\n", initialAngle, alignTime);
    }
}

INSTANTIATE_TEST_SUITE_P(
  TestAngles, YawTest,
  ::testing::Values(
      0, 45, -45, 90, 180, 270, 720+45
      ));

// A steady coordinated level turn in the body frame (X forward, Y left, Z up).
// Positive bank is right wing down, which turns the aircraft clockwise seen
// from above, i.e. negative about earth up.
struct CoordinatedTurn {
    vector3_t up;               // earth up
    vector3_t gyroRadS;
    vector3_t specificForceG;
};

static CoordinatedTurn coordinatedTurn(float bankDeg, float speedMs)
{
    const float bank = DEGREES_TO_RADIANS(bankDeg);
    const float turnRate = -G_ACCELERATION * tanf(bank) / speedMs;

    CoordinatedTurn turn;
    turn.up = {{ 0.0f, sinf(bank), cosf(bank) }};
    vector3Scale(&turn.gyroRadS, &turn.up, turnRate);
    // gravity reaction plus the centripetal acceleration: no side force, 1 / cos(bank) g on Z
    turn.specificForceG = {{ 0.0f, 0.0f, 1.0f / cosf(bank) }};
    return turn;
}

static bool testIsFixedWing;
static bool testGpsNewData;
static uint32_t testSensors = SENSOR_ACC;
static float testGyroDps[XYZ_AXIS_COUNT];

TEST(FlightImuWingTest, CentripetalRemovalLeavesGravityInACoordinatedTurn)
{
    acc.dev.acc_1G = 512;

    for (const float bankDeg : { 20.0f, -20.0f, 45.0f, -45.0f }) {
        const CoordinatedTurn turn = coordinatedTurn(bankDeg, 15.0f);
        vector3_t accAdc;
        vector3Scale(&accAdc, &turn.specificForceG, acc.dev.acc_1G);

        imuRemoveCentripetalAcc(&accAdc, &turn.gyroRadS, 15.0f);

        EXPECT_NEAR(0.0f, accAdc.x, 1e-3f) << bankDeg;
        EXPECT_NEAR(turn.up.y * acc.dev.acc_1G, accAdc.y, 1e-2f) << bankDeg;
        EXPECT_NEAR(turn.up.z * acc.dev.acc_1G, accAdc.z, 1e-2f) << bankDeg;
    }
}

// imu.c keeps the time of its previous update across tests
static timeUs_t timeUs;

class CoordinatedTurnTest : public ::testing::Test {
protected:
    static constexpr float bankDeg = 20.0f;
    static constexpr float speedMs = 15.0f;
    static constexpr float dt = 0.01f;

    void SetUp() override
    {
        imuConfigMutable()->imu_dcm_kp = 2500;
        imuConfigMutable()->imu_dcm_ki = 0;
        imuConfigure(0, 0);

        acc.dev.acc_1G = 512;
        acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;
        acc.isAccelUpdatedAtLeastOnce = true;
        vector3Zero(&mag.magADC);

        armingFlags = ARMED;
        stateFlags = GPS_FIX;
        gpsSol.numSat = 12;
        gpsSol.groundSpeed = speedMs * 100;
        testSensors = SENSOR_ACC | SENSOR_GPS;
        testIsFixedWing = true;
        autopilotWingConfigMutable()->cruiseSpeed = lrintf(speedMs * 10.0f);

        quaternion_from_axis_angle(&q, DEGREES_TO_RADIANS(bankDeg), 1, 0, 0);
        imuComputeRotationMatrix();
        imuUpdateEulerAngles();
    }

    void TearDown() override
    {
        armingFlags = 0;
        stateFlags = 0;
        testSensors = SENSOR_ACC;
        testIsFixedWing = false;
        memset(testGyroDps, 0, sizeof(testGyroDps));
        memset(gyro.gyroADCf, 0, sizeof(gyro.gyroADCf));
    }

    // The GPS reporting the track at seconds into the turn, through windEastMs of wind.
    void reportTrack(float seconds, float windEastMs)
    {
        const float headingRad = G_ACCELERATION * tanf(DEGREES_TO_RADIANS(bankDeg)) / speedMs * seconds;
        const float eastMs = speedMs * sinf(headingRad) + windEastMs;
        const float northMs = speedMs * cosf(headingRad);
        gpsSol.groundSpeed = lrintf(hypotf(eastMs, northMs) * 100.0f);
        gpsSol.groundCourse = lrintf(fmodf(RADIANS_TO_DEGREES(atan2f(eastMs, northMs)) + 360.0f, 360.0f) * 10.0f);
        gpsSol.llh.lat = lrintf(seconds * 1000.0f);
        testGpsNewData = true;
    }

    // fly the turn for the given time, the GPS reporting at 10 Hz; the worst roll and pitch errors from
    // settleS in, degrees
    void fly(float seconds, float *maxRollErrDeg, float *maxPitchErrDeg, float windEastMs = 0.0f, float settleS = 0.0f)
    {
        const CoordinatedTurn turn = coordinatedTurn(bankDeg, speedMs);
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            testGyroDps[axis] = RADIANS_TO_DEGREES(turn.gyroRadS.v[axis]);
            gyro.gyroADCf[axis] = testGyroDps[axis];
        }
        vector3Scale(&acc.accADC, &turn.specificForceG, acc.dev.acc_1G);
        acc.accMagnitude = vector3Norm(&acc.accADC) * acc.dev.acc_1G_rec;

        *maxRollErrDeg = 0.0f;
        *maxPitchErrDeg = 0.0f;
        float headingChangeDeg = 0.0f;
        int steps = 0;
        for (float t = 0.0f; t < seconds; t += dt) {
            const int16_t previousYaw = attitude.values.yaw;
            if (steps++ % 10 == 0) {
                reportTrack(t, windEastMs);
            }
            timeUs += lrintf(dt * 1e6f);
            imuUpdateAttitude(timeUs);
            headingChangeDeg += remainderf(attitude.values.yaw - previousYaw, 3600.0f) / 10.0f;
            if (t >= settleS) {
                *maxRollErrDeg = fmaxf(*maxRollErrDeg, fabsf(attitude.values.roll / 10.0f - bankDeg));
                *maxPitchErrDeg = fmaxf(*maxPitchErrDeg, fabsf(attitude.values.pitch / 10.0f));
            }
        }
        // a right bank turns clockwise: heading increases at g tan(bank) / V
        const float expectedHeadingChangeDeg = RADIANS_TO_DEGREES(G_ACCELERATION * tanf(DEGREES_TO_RADIANS(bankDeg)) / speedMs) * seconds;
        EXPECT_NEAR(expectedHeadingChangeDeg, headingChangeDeg, 0.1f * expectedHeadingChangeDeg);
    }
};

TEST_F(CoordinatedTurnTest, WingHoldsTheBankOnGpsSpeed)
{
    float maxRollErr, maxPitchErr;
    fly(60.0f, &maxRollErr, &maxPitchErr);

    EXPECT_LT(maxRollErr, 1.0f);
    EXPECT_LT(maxPitchErr, 1.0f);
}

TEST_F(CoordinatedTurnTest, WingHoldsTheBankTurningThroughAWind)
{
    float maxRollErr, maxPitchErr;
    // from a few seconds in, once the turn has shown the airspeed
    fly(60.0f, &maxRollErr, &maxPitchErr, 5.0f, 5.0f);

    EXPECT_LT(maxRollErr, 1.0f);
    EXPECT_LT(maxPitchErr, 1.0f);
}

TEST_F(CoordinatedTurnTest, TurningThroughAWindTheSpeedIsTheAirspeed)
{
    const CoordinatedTurn turn = coordinatedTurn(bankDeg, speedMs);
    float slowest = 1e3f, fastest = 0.0f, slowestOverGround = 1e3f, fastestOverGround = 0.0f;
    timeUs_t nowUs = 0;
    for (int i = 0; i < 6000; i++) {
        const float t = i * dt;
        if (i % 10 == 0) {
            reportTrack(t, 5.0f);
        }
        nowUs += lrintf(dt * 1e6f);
        const float speed = imuWingTurnSpeedMs(nowUs, dt, &turn.gyroRadS);
        if (t > 5.0f) {
            slowest = fminf(slowest, speed);
            fastest = fmaxf(fastest, speed);
            slowestOverGround = fminf(slowestOverGround, gpsSol.groundSpeed * 0.01f);
            fastestOverGround = fmaxf(fastestOverGround, gpsSol.groundSpeed * 0.01f);
        }
    }
    EXPECT_NEAR(speedMs, slowest, 0.5f);
    EXPECT_NEAR(speedMs, fastest, 0.5f);
    EXPECT_GT(fastestOverGround - slowestOverGround, 9.0f);
}

TEST_F(CoordinatedTurnTest, RollingFromOneTurnIntoAnotherTheSpeedStaysNearTheAirspeed)
{
    // turning at 0.3 rad/s one way, then the other, the GPS reporting the track 100 ms late through 5 m/s of wind
    const CoordinatedTurn turn = coordinatedTurn(bankDeg, speedMs);
    const vector3_t up = turn.up;
    float headingRad = 0.0f;
    float delayedEastMs = 0.0f, delayedNorthMs = speedMs;
    float worstErr = 0.0f;
    timeUs_t nowUs = 0;
    for (int i = 0; i < 3000; i++) {
        const float t = i * dt;
        const float turnRadS = (fmodf(t, 10.0f) < 5.0f) ? 0.3f : -0.3f;
        headingRad += turnRadS * dt;
        if (i % 10 == 0) {
            gpsSol.groundSpeed = lrintf(hypotf(delayedEastMs, delayedNorthMs) * 100.0f);
            gpsSol.groundCourse = lrintf(fmodf(RADIANS_TO_DEGREES(atan2f(delayedEastMs, delayedNorthMs)) + 360.0f, 360.0f) * 10.0f);
            gpsSol.llh.lat = i;
            testGpsNewData = true;
            delayedEastMs = speedMs * sinf(headingRad) + 5.0f;
            delayedNorthMs = speedMs * cosf(headingRad);
        }
        vector3_t gyroRadS;
        vector3Scale(&gyroRadS, &up, -turnRadS);
        nowUs += lrintf(dt * 1e6f);
        const float speed = imuWingTurnSpeedMs(nowUs, dt, &gyroRadS);
        if (t > 5.0f) {
            worstErr = fmaxf(worstErr, fabsf(speed - speedMs));
        }
    }
    EXPECT_LT(worstErr, 3.0f);
}

TEST_F(CoordinatedTurnTest, TurningSlowlyANoisyGpsWithAGlitchDoesNotThrowTheSpeedOff)
{
    // at 10 deg/s, where the turn just shows the airspeed, through 5 m/s of wind; the GPS velocity has
    // 0.1 m/s of noise and once a second jumps 5 m/s for a fix
    const float turnRadS = DEGREES_TO_RADIANS(10.0f);
    const vector3_t gyroRadS = {{ 0.0f, 0.0f, -turnRadS }};
    uint32_t seed = 1;
    const auto noiseMs = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return 0.1f * ((seed >> 8) * (2.0f / 16777216.0f) - 1.0f) * sqrtf(3.0f);
    };
    float worstErr = 0.0f;
    timeUs_t nowUs = 0;
    for (int i = 0; i < 6000; i++) {
        const float t = i * dt;
        if (i % 10 == 0) {
            const float headingRad = turnRadS * t;
            const float glitchMs = (i % 100 == 50) ? 5.0f : 0.0f;
            const float eastMs = speedMs * sinf(headingRad) + 5.0f + noiseMs() + glitchMs;
            const float northMs = speedMs * cosf(headingRad) + noiseMs();
            gpsSol.groundSpeed = lrintf(hypotf(eastMs, northMs) * 100.0f);
            gpsSol.groundCourse = lrintf(fmodf(RADIANS_TO_DEGREES(atan2f(eastMs, northMs)) + 360.0f, 360.0f) * 10.0f);
            gpsSol.llh.lat = i;
            testGpsNewData = true;
        }
        nowUs += lrintf(dt * 1e6f);
        const float speed = imuWingTurnSpeedMs(nowUs, dt, &gyroRadS);
        if (t > 5.0f) {
            worstErr = fmaxf(worstErr, fabsf(speed - speedMs));
        }
    }
    EXPECT_LT(worstErr, 3.0f);
}

TEST_F(CoordinatedTurnTest, AGpsTrackNoTurnCouldGiveReadsNoFasterThanTwiceTheCruiseSpeed)
{
    // the track running away at 1 g while the heading turns at 10 deg/s
    const vector3_t gyroRadS = {{ 0.0f, 0.0f, -DEGREES_TO_RADIANS(10.0f) }};
    timeUs_t nowUs = 0;
    float speed = 0.0f;
    for (int i = 0; i < 300; i++) {
        if (i % 10 == 0) {
            gpsSol.groundSpeed = lrintf(G_ACCELERATION * i * dt * 100.0f);
            gpsSol.groundCourse = 900;
            gpsSol.llh.lat = i;
            testGpsNewData = true;
        }
        nowUs += lrintf(dt * 1e6f);
        speed = imuWingTurnSpeedMs(nowUs, dt, &gyroRadS);
    }
    EXPECT_LE(speed, 2.0f * speedMs);
}

TEST_F(CoordinatedTurnTest, WithTheGpsQuietTheSpeedIsTheLastGroundspeed)
{
    const CoordinatedTurn turn = coordinatedTurn(bankDeg, speedMs);
    timeUs_t nowUs = 0;
    float speed = 0.0f;
    for (int i = 0; i < 2000; i++) {
        if (i % 10 == 0 && i < 1500) {
            reportTrack(i * dt, 5.0f);
        }
        nowUs += lrintf(dt * 1e6f);
        speed = imuWingTurnSpeedMs(nowUs, dt, &turn.gyroRadS);
    }
    EXPECT_FLOAT_EQ(gpsSol.groundSpeed * 0.01f, speed);
}

TEST_F(CoordinatedTurnTest, WithTheGpsRepeatingItsLastFixTheSpeedIsTheLastGroundspeed)
{
    const CoordinatedTurn turn = coordinatedTurn(bankDeg, speedMs);
    timeUs_t nowUs = 0;
    float speed = 0.0f;
    for (int i = 0; i < 2000; i++) {
        if (i % 10 == 0) {
            if (i < 1500) {
                reportTrack(i * dt, 5.0f);
            } else {
                testGpsNewData = true;
            }
        }
        nowUs += lrintf(dt * 1e6f);
        speed = imuWingTurnSpeedMs(nowUs, dt, &turn.gyroRadS);
    }
    EXPECT_FLOAT_EQ(gpsSol.groundSpeed * 0.01f, speed);
}

TEST_F(CoordinatedTurnTest, FlyingStraightTheSpeedIsTheGroundspeed)
{
    const vector3_t still = {{ 0.0f, 0.0f, 0.0f }};
    float speed = 0.0f;
    timeUs_t nowUs = 0;
    for (int i = 0; i < 1000; i++) {
        if (i % 10 == 0) {
            reportTrack(0.0f, 5.0f);
            gpsSol.llh.lat = i;
        }
        nowUs += lrintf(dt * 1e6f);
        speed = imuWingTurnSpeedMs(nowUs, dt, &still);
    }
    EXPECT_NEAR(hypotf(5.0f, speedMs), speed, 0.01f);
}

TEST_F(CoordinatedTurnTest, WingHoldsTheBankOnTheCruiseSpeedWithoutGps)
{
    stateFlags = 0;

    float maxRollErr, maxPitchErr;
    fly(60.0f, &maxRollErr, &maxPitchErr);

    EXPECT_LT(maxRollErr, 1.0f);
    EXPECT_LT(maxPitchErr, 1.0f);
}

// STUBS

extern "C" {
    extern boxBitmask_t rcModeActivationMask;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    gyro_t gyro;
    acc_t acc;
    mag_t mag;

    gpsSolutionData_t gpsSol;

    bool gpsHasNewData(uint16_t *)
    {
        const bool fresh = testGpsNewData;
        testGpsNewData = false;
        return fresh;
    }

    uint8_t debugMode;
    int16_t debug[DEBUG16_VALUE_COUNT];

    uint8_t stateFlags;
    uint16_t flightModeFlags;
    uint8_t armingFlags;

    pidProfile_t *currentPidProfile;

    uint16_t enableFlightMode(flightModeFlags_e mask) {
        return flightModeFlags |= (mask);
    }

    uint16_t disableFlightMode(flightModeFlags_e mask) {
        return flightModeFlags &= ~(mask);
    }

    bool sensors(uint32_t mask) {
        return mask & testSensors;
    };

    uint32_t millis(void) { return 0; }
    uint32_t micros(void) { return 0; }

    bool compassEnabledAndCalibrated(void) { return true; }
    bool baroIsCalibrated(void) { return true; }
    void performBaroCalibrationCycle(void) {}
    float baroCalculateAltitude(void) { return 0; }
    bool gyroGetAccumulationAverage(float *) { return false; }
    bool accGetAccumulationAverage(float *) { return false; }
    void mixerSetThrottleAngleCorrection(int) {};
    bool gpsRescueIsRunning(void) { return false; }
    bool isFixedWing(void) { return testIsFixedWing; }
    void pinioBoxTaskControl(void) {}
    void schedulerIgnoreTaskExecTime(void) {}
    void schedulerIgnoreTaskStateTime(void) {}
    void schedulerSetNextStateTime(timeDelta_t) {}
    bool schedulerGetIgnoreTaskExecTime() { return false; }
    float gyroGetFilteredDownsampled(int axis) { return testGyroDps[axis]; }
    float baroUpsampleAltitude()  { return 0.0f; }
    float getBaroAltitude(void) { return 3000.0f; }
    float getRcDeflectionAbs(int) { return 0.0f; }

    void positionEstimatorInit(void) { }
    void positionEstimatorUpdate(void) { }
    void positionEstimatorResetZ(void) { }
    bool positionEstimatorIsValidZ(void) { return false; }
    float positionEstimatorGetAltitudeCm(void) { return 0.0f; }
    float positionEstimatorGetVerticalVelocity(void) { return 0.0f; }
    float positionEstimatorGetVerticalAcceleration(void) { return 0.0f; }
    float positionEstimatorGetTrustZ(void) { return 0.0f; }
    static positionEstimate3d_t stubEstimate = {};
    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &stubEstimate; }
}
