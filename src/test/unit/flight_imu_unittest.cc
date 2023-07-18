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
    
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/imu.h"
    #include "flight/position.h"

    
    #include "io/gps.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"

    void imuComputeRotationMatrix(void);
    void imuUpdateEulerAngles(void);
    void imuMahonyAHRSupdate(float dt,
                             float gx, float gy, float gz,
                             bool useAcc, float ax, float ay, float az,
                             float headingErrMag, float headingErrCog,
                             const float dcmKpGain);
    float imuCalcMagErr(void);
    float imuCalcCourseErr(float courseOverGround);
    extern quaternion q;
    extern float rMat[3][3];
    extern bool attitudeIsEstablished;

    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(barometerConfig_t, barometerConfig, PG_BAROMETER_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

    PG_RESET_TEMPLATE(featureConfig_t, featureConfig,
        .enabledFeatures = 0
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

const float sqrt2over2 = sqrtf(2) / 2.0f;

void quaternion_from_axis_angle(quaternion* q, float angle, float x, float y, float z) {
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

    EXPECT_FLOAT_EQ(1.0f, rMat[0][0]);
    EXPECT_FLOAT_EQ(0.0f, rMat[0][1]);
    EXPECT_FLOAT_EQ(0.0f, rMat[0][2]);
    EXPECT_FLOAT_EQ(0.0f, rMat[1][0]);
    EXPECT_FLOAT_EQ(1.0f, rMat[1][1]);
    EXPECT_FLOAT_EQ(0.0f, rMat[1][2]);
    EXPECT_FLOAT_EQ(0.0f, rMat[2][0]);
    EXPECT_FLOAT_EQ(0.0f, rMat[2][1]);
    EXPECT_FLOAT_EQ(1.0f, rMat[2][2]);

    // 90 degrees around Z axis
    q.w = sqrt2over2;
    q.x = 0.0f;
    q.y = 0.0f;
    q.z = sqrt2over2;

    imuComputeRotationMatrix();

    EXPECT_NEAR(0.0f, rMat[0][0], TOL);
    EXPECT_NEAR(-1.0f, rMat[0][1], TOL);
    EXPECT_NEAR(0.0f, rMat[0][2], TOL);
    EXPECT_NEAR(1.0f, rMat[1][0], TOL);
    EXPECT_NEAR(0.0f, rMat[1][1], TOL);
    EXPECT_NEAR(0.0f, rMat[1][2], TOL);
    EXPECT_NEAR(0.0f, rMat[2][0], TOL);
    EXPECT_NEAR(0.0f, rMat[2][1], TOL);
    EXPECT_NEAR(1.0f, rMat[2][2], TOL);

    // 60 degrees around X axis
    q.w = 0.866f;
    q.x = 0.5f;
    q.y = 0.0f;
    q.z = 0.0f;

    imuComputeRotationMatrix();

    EXPECT_NEAR(1.0f, rMat[0][0], TOL);
    EXPECT_NEAR(0.0f, rMat[0][1], TOL);
    EXPECT_NEAR(0.0f, rMat[0][2], TOL);
    EXPECT_NEAR(0.0f, rMat[1][0], TOL);
    EXPECT_NEAR(0.5f, rMat[1][1], TOL);
    EXPECT_NEAR(-0.866f, rMat[1][2], TOL);
    EXPECT_NEAR(0.0f, rMat[2][0], TOL);
    EXPECT_NEAR(0.866f, rMat[2][1], TOL);
    EXPECT_NEAR(0.5f, rMat[2][2], TOL);
}

TEST(FlightImuTest, TestUpdateEulerAngles)
{
    // No rotation
    memset(rMat, 0.0, sizeof(float) * 9);

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(0, attitude.values.yaw);

    // 45 degree yaw
    memset(rMat, 0.0, sizeof(float) * 9);
    rMat[0][0] = sqrt2over2;
    rMat[0][1] = sqrt2over2;
    rMat[1][0] = -sqrt2over2;
    rMat[1][1] = sqrt2over2;

    imuUpdateEulerAngles();

    EXPECT_EQ(0, attitude.values.roll);
    EXPECT_EQ(0, attitude.values.pitch);
    EXPECT_EQ(450, attitude.values.yaw);
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
    memset(rMat, 0.0, sizeof(float) * 9);

    // when
    imuComputeRotationMatrix();

    // expect
    EXPECT_FALSE(isUpright());

    // given
    rMat[0][0] = r1;
    rMat[0][2] = r2;
    rMat[2][0] = -r2;
    rMat[2][2] = r1;

    // when
    imuComputeRotationMatrix();

    // expect
    EXPECT_FALSE(isUpright());

    // given
    memset(rMat, 0.0, sizeof(float) * 9);

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

    for (int i = 0; i < 3; i++)
        mag.magADC[i] = magBF.v[i];

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

// STUBS

extern "C" {
    extern boxBitmask_t rcModeActivationMask;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    gyro_t gyro;
    acc_t acc;
    mag_t mag;

    gpsSolutionData_t gpsSol;

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
        return mask & SENSOR_ACC;
    };

    uint32_t millis(void) { return 0; }
    uint32_t micros(void) { return 0; }

    bool compassIsHealthy(void) { return true; }
    bool baroIsCalibrated(void) { return true; }
    void performBaroCalibrationCycle(void) {}
    float baroCalculateAltitude(void) { return 0; }
    bool gyroGetAccumulationAverage(float *) { return false; }
    bool accGetAccumulationAverage(float *) { return false; }
    void mixerSetThrottleAngleCorrection(int) {};
    bool gpsRescueIsRunning(void) { return false; }
    bool isFixedWing(void) { return false; }
    void pinioBoxTaskControl(void) {}
    void schedulerIgnoreTaskExecTime(void) {}
    void schedulerIgnoreTaskStateTime(void) {}
    void schedulerSetNextStateTime(timeDelta_t) {}
    bool schedulerGetIgnoreTaskExecTime() { return false; }
    float gyroGetFilteredDownsampled(int) { return 0.0f; }
    float baroUpsampleAltitude()  { return 0.0f; }
    float pt2FilterGain(float, float)  { return 0.0f; }
    float getBaroAltitude(void) { return 3000.0f; }
    float gpsRescueGetImuYawCogGain(void) { return 1.0f; }
    float getRcDeflectionAbs(int) { return 0.0f; }

    void pt2FilterInit(pt2Filter_t *baroDerivativeLpf, float) {
        UNUSED(baroDerivativeLpf);
    }
    float pt2FilterApply(pt2Filter_t *baroDerivativeLpf, float) {
        UNUSED(baroDerivativeLpf);
        return 0.0f;
    }
}
