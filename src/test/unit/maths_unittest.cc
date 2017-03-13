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

/*
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
*/
#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
/*
TEST(MathsUnittest, TestFastTrigonometrySinCos)
{
    double sinError = 0;
    for (float x = -10 * M_PI; x < 10 * M_PI; x += M_PI / 300) {
        double approxResult = sin_approx(x);
        double libmResult = sinf(x);
        sinError = MAX(sinError, fabs(approxResult - libmResult));
    }
    printf("sin_approx maximum absolute error = %e\n", sinError);
    EXPECT_LE(sinError, 3e-6);

    double cosError = 0;
    for (float x = -10 * M_PI; x < 10 * M_PI; x += M_PI / 300) {
        double approxResult = cos_approx(x);
        double libmResult = cosf(x);
        cosError = MAX(cosError, fabs(approxResult - libmResult));
    }
    printf("cos_approx maximum absolute error = %e\n", cosError);
    EXPECT_LE(cosError, 3e-6);
}
*/

TEST(MathsUnittest, TestFastTrigonometryATan2)
{
    double error = 0;
    for (float x = -1.0f; x < 1.0f; x += 0.01) {
        for (float y = -1.0f; x < 1.0f; x += 0.001) {
            double approxResult = atan2_approx(y, x);
            double libmResult = atan2f(y, x);
            error = MAX(error, fabs(approxResult - libmResult));
        }
    }
    printf("atan2_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0f);
    EXPECT_LE(error, 1e-6);
}

TEST(MathsUnittest, TestFastTrigonometryACos)
{
    double error = 0;
    for (float x = -1.0f; x < 1.0f; x += 0.001) {
        double approxResult = acos_approx(x);
        double libmResult = acos(x);
        error = MAX(error, fabs(approxResult - libmResult));
    }
    printf("acos_approx maximum absolute error = %e rads (%e degree)\n", error, error / M_PI * 180.0f);
    EXPECT_LE(error, 1e-4);
}
#endif
