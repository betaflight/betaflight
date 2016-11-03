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

#include <math.h>

#define BARO

extern "C" {
    #include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(MathsUnittest, TestConstrain)
{
    // Within bounds
    EXPECT_EQ(constrain(0, 0, 0), 0);
    EXPECT_EQ(constrain(1, 1, 1), 1);
    EXPECT_EQ(constrain(1, 0, 2), 1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(1, 1, 2), 1);
    // Equal to top bound.
    EXPECT_EQ(constrain(1, 0, 1), 1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(1, 1, 1), 1);

    // Above top bound.
    EXPECT_EQ(constrain(2, 0, 1), 1);
    // Below bottom bound.
    EXPECT_EQ(constrain(0, 1, 2), 1);
}

TEST(MathsUnittest, TestConstrainNegatives)
{
    // Within bounds.
    EXPECT_EQ(constrain(-1, -1, -1), -1);
    EXPECT_EQ(constrain(-1, -2, 0), -1);

    // Equal to bottom bound.
    EXPECT_EQ(constrain(-1, -1, 0), -1);
    // Equal to top bound.
    EXPECT_EQ(constrain(-1, -2, -1), -1);

    // Equal to both bottom and top bound.
    EXPECT_EQ(constrain(-1, -1, -1), -1);

    // Above top bound.
    EXPECT_EQ(constrain(-1, -3, -2), -2);
    // Below bottom bound.
    EXPECT_EQ(constrain(-3, -2, -1), -2);
}

TEST(MathsUnittest, TestConstrainf)
{
    // Within bounds.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 2.0f), 1.0f);

    // Equal to bottom bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 2.0f), 1.0f);
    // Equal to top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 0.0f, 1.0f), 1.0f);

    // Equal to both bottom and top bound.
    EXPECT_FLOAT_EQ(constrainf(1.0f, 1.0f, 1.0f), 1.0f);

    // Above top bound.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bottom bound.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);

    // Above bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(2.0f, 0.0f, 1.0f), 1.0f);
    // Below bouth bounds.
    EXPECT_FLOAT_EQ(constrainf(0, 1.0f, 2.0f), 1.0f);
}

TEST(MathsUnittest, TestDegreesToRadians)
{
    EXPECT_FLOAT_EQ(degreesToRadians(0), 0.0f);
    EXPECT_FLOAT_EQ(degreesToRadians(90), 0.5f * M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(180), M_PIf);
    EXPECT_FLOAT_EQ(degreesToRadians(-180), - M_PIf);
}

TEST(MathsUnittest, TestApplyDeadband)
{
    EXPECT_EQ(applyDeadband(0, 0), 0);
    EXPECT_EQ(applyDeadband(1, 0), 1);
    EXPECT_EQ(applyDeadband(-1, 0), -1);

    EXPECT_EQ(applyDeadband(0, 10), 0);
    EXPECT_EQ(applyDeadband(1, 10), 0);
    EXPECT_EQ(applyDeadband(10, 10), 0);

    EXPECT_EQ(applyDeadband(11, 10), 1);
    EXPECT_EQ(applyDeadband(-11, 10), -1);
}

void expectVectorsAreEqual(struct fp_vector *a, struct fp_vector *b)
{
    EXPECT_FLOAT_EQ(a->X, b->X);
    EXPECT_FLOAT_EQ(a->Y, b->Y);
    EXPECT_FLOAT_EQ(a->Z, b->Z);
}

TEST(MathsUnittest, TestRotateVectorWithNoAngle)
{
    fp_vector vector = {1.0f, 0.0f, 0.0f};
    fp_angles_t euler_angles = {.raw={0.0f, 0.0f, 0.0f}};

    rotateV(&vector, &euler_angles);
    fp_vector expected_result = {1.0f, 0.0f, 0.0f};

    expectVectorsAreEqual(&vector, &expected_result);
}

TEST(MathsUnittest, TestRotateVectorAroundAxis)
{
    // Rotate a vector <1, 0, 0> around an each axis x y and z.
    fp_vector vector = {1.0f, 0.0f, 0.0f};
    fp_angles_t euler_angles = {.raw={90.0f, 0.0f, 0.0f}};

    rotateV(&vector, &euler_angles);
    fp_vector expected_result = {1.0f, 0.0f, 0.0f};

    expectVectorsAreEqual(&vector, &expected_result);
}

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
TEST(MathsUnittest, TestFastTrigonometrySinCos)
{
    EXPECT_NEAR(sin_approx(0.0f),                        0.0f, 1e-6);
    EXPECT_NEAR(sin_approx(M_PIf / 2),                   1.0f, 1e-6);
    EXPECT_NEAR(sin_approx(-M_PIf / 2),                 -1.0f, 1e-6);
    EXPECT_NEAR(sin_approx(M_PIf),                       0.0f, 1e-6);
    EXPECT_NEAR(sin_approx(-M_PIf),                      0.0f, 1e-6);
    EXPECT_NEAR(sin_approx(3 * M_PIf / 2),              -1.0f, 1e-6);
    EXPECT_NEAR(sin_approx(-3 * M_PIf / 2),              1.0f, 1e-6);
    EXPECT_NEAR(sin_approx(2 * M_PIf),                   0.0f, 1e-6);
    EXPECT_NEAR(sin_approx(-2 * M_PIf),                  0.0f, 1e-6);
    EXPECT_NEAR(sin_approx(3 * M_PIf / 4),               0.707106781f, 1e-6);
    EXPECT_NEAR(sin_approx(-3 * M_PIf / 4),             -0.707106781f, 1e-6);
    EXPECT_NEAR(sin_approx(2 * M_PIf + 3 * M_PIf / 4),   0.707106781f, 1e-6);
    EXPECT_NEAR(sin_approx(-2 * M_PIf - 3 * M_PIf / 4), -0.707106781f, 1e-6);

    EXPECT_NEAR(cos_approx(0.0f),                        1.0f, 1e-6);
    EXPECT_NEAR(cos_approx(M_PIf / 2),                   0.0f, 1e-6);
    EXPECT_NEAR(cos_approx(-M_PIf / 2),                  0.0f, 1e-6);
    EXPECT_NEAR(cos_approx(M_PIf),                      -1.0f, 1e-6);
    EXPECT_NEAR(cos_approx(-M_PIf),                     -1.0f, 1e-6);
    EXPECT_NEAR(cos_approx(3 * M_PIf / 2),               0.0f, 1e-6);
    EXPECT_NEAR(cos_approx(-3 * M_PIf / 2),              0.0f, 1e-6);
    EXPECT_NEAR(cos_approx(2 * M_PIf),                   1.0f, 1e-6);
    EXPECT_NEAR(cos_approx(-2 * M_PIf),                  1.0f, 1e-6);
    EXPECT_NEAR(cos_approx(3 * M_PIf / 4),              -0.707106781f, 1e-6);
    EXPECT_NEAR(cos_approx(-3 * M_PIf / 4),             -0.707106781f, 1e-6);
    EXPECT_NEAR(cos_approx(2 * M_PIf + 3 * M_PIf / 4),  -0.707106781f, 1e-6);
    EXPECT_NEAR(cos_approx(-2 * M_PIf - 3 * M_PIf / 4), -0.707106781f, 1e-6);
}

TEST(MathsUnittest, TestFastTrigonometryATan2)
{
    EXPECT_NEAR(atan2_approx(1, 1),          M_PIf / 4, 1e-6);
    EXPECT_NEAR(atan2_approx(-1, 1),        -M_PIf / 4, 1e-6);
    EXPECT_NEAR(atan2_approx(1, -1),     3 * M_PIf / 4, 1e-6);
    EXPECT_NEAR(atan2_approx(-1, -1),   -3 * M_PIf / 4, 1e-6);
    EXPECT_NEAR(atan2_approx(0, 1),                  0, 1e-6);
    EXPECT_NEAR(atan2_approx(0, -1),             M_PIf, 1e-6);
    EXPECT_NEAR(atan2_approx(1, 0),          M_PIf / 2, 1e-6);
    EXPECT_NEAR(atan2_approx(-1, 0),        -M_PIf / 2, 1e-6);
}

TEST(MathsUnittest, TestFastTrigonometryACos)
{
    EXPECT_NEAR(acos_approx(0.0f),              M_PIf / 2, 1e-4);
    EXPECT_NEAR(acos_approx(1.0f),                      0, 1e-4);
    EXPECT_NEAR(acos_approx(-1.0f),                 M_PIf, 1e-4);
    EXPECT_NEAR(acos_approx(0.707106781f),      M_PIf / 4, 1e-4);
    EXPECT_NEAR(acos_approx(-0.707106781f), 3 * M_PIf / 4, 1e-4);
}

/*
TEST(MathsUnittest, TestSensorScaleUnitTest)
{
    sensorCalibrationState_t calState;
    float result[3];

    int16_t samples[6][3] = {
        {  2896,  2896,      0 },
        { -2897,  2896,      0 },
        {     0,  4096,      0 },
        {     0, -4096,      0 },
        {     0,  2895,  -2897 },
        {     0,     0,  -4096 } };

    // Given
    sensorCalibrationResetState(&calState);
    sensorCalibrationPushSampleForScaleCalculation(&calState, 0, samples[0], 4096 );
    sensorCalibrationPushSampleForScaleCalculation(&calState, 0, samples[1], 4096 );
    sensorCalibrationPushSampleForScaleCalculation(&calState, 1, samples[2], 4096 );
    sensorCalibrationPushSampleForScaleCalculation(&calState, 1, samples[3], 4096 );
    sensorCalibrationPushSampleForScaleCalculation(&calState, 2, samples[4], 4096 );
    sensorCalibrationPushSampleForScaleCalculation(&calState, 2, samples[5], 4096 );
    sensorCalibrationSolveForScale(&calState, result);
    
    EXPECT_NEAR(result[0], 1, 1e-4);
    EXPECT_NEAR(result[1], 1, 1e-4);
    EXPECT_NEAR(result[2], 1, 1e-4);
}
*/
#endif
