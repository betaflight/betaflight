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

#include "math.h"
#include "stdint.h"
#include "time.h"

extern "C" {
#include "common/axis.h"
#include "common/sensor_alignment.h"
#include "drivers/sensor.h"
#include "sensors/boardalignment.h"
#include "sensors/sensors.h"
}

#include "gtest/gtest.h"

/*
 * This test file contains an independent method of rotating a vector.
 * The output of alignSensor() is compared to the output of the test
 * rotation method.
 *
 * For each alignment condition (LEGACY_ALIGN_CW0, CW90, etc) the source vector under
 * test is set to a unit vector along each axis (x-axis, y-axis, z-axis)
 * plus one additional random vector is tested.
 */

#define DEG2RAD 0.01745329251

static void rotateVector(int32_t mat[3][3], float vec[3], float *out)
{
    float tmp[3];

    for(int i=0; i<3; i++) {
        tmp[i] = 0;
        for(int j=0; j<3; j++) {
            tmp[i] += mat[j][i] * vec[j];
        }
    }

    out[0]=tmp[0];
    out[1]=tmp[1];
    out[2]=tmp[2];

}

//static void initXAxisRotation(int32_t mat[][3], int32_t angle)
//{
//    mat[0][0] =  1;
//    mat[0][1] =  0;
//    mat[0][2] =  0;
//    mat[1][0] =  0;
//    mat[1][1] =  cos(angle*DEG2RAD);
//    mat[1][2] = -sin(angle*DEG2RAD);
//    mat[2][0] =  0;
//    mat[2][1] =  sin(angle*DEG2RAD);
//    mat[2][2] =  cos(angle*DEG2RAD);
//}

static void initYAxisRotation(int32_t mat[][3], int32_t angle)
{
    mat[0][0] =  cos(angle*DEG2RAD);
    mat[0][1] =  0;
    mat[0][2] =  sin(angle*DEG2RAD);
    mat[1][0] =  0;
    mat[1][1] =  1;
    mat[1][2] =  0;
    mat[2][0] = -sin(angle*DEG2RAD);
    mat[2][1] =  0;
    mat[2][2] =  cos(angle*DEG2RAD);
}

static void initZAxisRotation(int32_t mat[][3], int32_t angle)
{
    mat[0][0] =  cos(angle*DEG2RAD);
    mat[0][1] = -sin(angle*DEG2RAD);
    mat[0][2] =  0;
    mat[1][0] =  sin(angle*DEG2RAD);
    mat[1][1] =  cos(angle*DEG2RAD);
    mat[1][2] =  0;
    mat[2][0] =  0;
    mat[2][1] =  0;
    mat[2][2] =  1;
}

#define TOL 1e-5 // TOLERANCE

static void testCW(legacy_sensor_align_e rotation, int32_t angle)
{
    float src[XYZ_AXIS_COUNT];
    float test[XYZ_AXIS_COUNT];

    // unit vector along x-axis
    src[X] = 1;
    src[Y] = 0;
    src[Z] = 0;

    int32_t matrix[3][3];
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, src, test);

    alignSensors(src, rotation);
    EXPECT_NEAR(test[X], src[X], TOL) << "X-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "X-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "X-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

    // unit vector along y-axis
    src[X] = 0;
    src[Y] = 1;
    src[Z] = 0;

    rotateVector(matrix, src, test);
    alignSensors(src, rotation);
    EXPECT_NEAR(test[X], src[X], TOL) << "Y-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Y-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Y-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

    // unit vector along z-axis
    src[X] = 0;
    src[Y] = 0;
    src[Z] = 1;

    rotateVector(matrix, src, test);
    alignSensors(src, rotation);
    EXPECT_NEAR(test[X], src[X], TOL) << "Z-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Z-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Z-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

    // random vector to test
    src[X] = rand() % 5;
    src[Y] = rand() % 5;
    src[Z] = rand() % 5;

    rotateVector(matrix, src, test);
    alignSensors(src,  rotation);
    EXPECT_NEAR(test[X], src[X], TOL) << "Random alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Random alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Random alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];
}

/*
 * Since the order of flip and rotation matters, these tests make the
 * assumption that the 'flip' occurs first, followed by clockwise rotation
 */
static void testCWFlip(legacy_sensor_align_e rotation, int32_t angle)
{
    float src[XYZ_AXIS_COUNT];
    float test[XYZ_AXIS_COUNT];

    // unit vector along x-axis
    src[X] = 1;
    src[Y] = 0;
    src[Z] = 0;

    int32_t matrix[3][3];
    initYAxisRotation(matrix, 180);
    rotateVector(matrix, src, test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, test, test);

    alignSensors(src, rotation);

    EXPECT_NEAR(test[X], src[X], TOL) << "X-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "X-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "X-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

    // unit vector along y-axis
    src[X] = 0;
    src[Y] = 1;
    src[Z] = 0;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, src, test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, test, test);

    alignSensors(src, rotation);

    EXPECT_NEAR(test[X], src[X], TOL) << "Y-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Y-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Y-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

    // unit vector along z-axis
    src[X] = 0;
    src[Y] = 0;
    src[Z] = 1;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, src, test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, test, test);

    alignSensors(src, rotation);

    EXPECT_NEAR(test[X], src[X], TOL) << "Z-Unit alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Z-Unit alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Z-Unit alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];

     // random vector to test
    src[X] = rand() % 5;
    src[Y] = rand() % 5;
    src[Z] = rand() % 5;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, src, test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, test, test);

    alignSensors(src, rotation);

    EXPECT_NEAR(test[X], src[X], TOL) << "Random alignment does not match in X-Axis. " << test[X] << " " << src[X];
    EXPECT_NEAR(test[Y], src[Y], TOL) << "Random alignment does not match in Y-Axis. " << test[Y] << " " << src[Y];
    EXPECT_NEAR(test[Z], src[Z], TOL) << "Random alignment does not match in Z-Axis. " << test[Z] << " " << src[Z];
}


TEST(AlignSensorTest, ClockwiseZeroDegrees)
{
    srand(time(NULL));
    testCW(LEGACY_ALIGN_CW0_DEG, 0);
}

TEST(AlignSensorTest, ClockwiseNinetyDegrees)
{
    testCW(LEGACY_ALIGN_CW90_DEG, 90);
}

TEST(AlignSensorTest, ClockwiseOneEightyDegrees)
{
    testCW(LEGACY_ALIGN_CW180_DEG, 180);
}

TEST(AlignSensorTest, ClockwiseTwoSeventyDegrees)
{
    testCW(LEGACY_ALIGN_CW270_DEG, 270);
}

TEST(AlignSensorTest, ClockwiseZeroDegreesFlip)
{
    testCWFlip(LEGACY_ALIGN_CW0_DEG_FLIP, 0);
}

TEST(AlignSensorTest, ClockwiseNinetyDegreesFlip)
{
    testCWFlip(LEGACY_ALIGN_CW90_DEG_FLIP, 90);
}

TEST(AlignSensorTest, ClockwiseOneEightyDegreesFlip)
{
    testCWFlip(LEGACY_ALIGN_CW180_DEG_FLIP, 180);
}

TEST(AlignSensorTest, ClockwiseTwoSeventyDegreesFlip)
{
    testCWFlip(LEGACY_ALIGN_CW270_DEG_FLIP, 270);
}
