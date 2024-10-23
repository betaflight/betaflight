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

#include <math.h>
#include <stdint.h>
#include <time.h>

extern "C" {
#include "common/axis.h"
#include "common/sensor_alignment.h"
#include "common/sensor_alignment_impl.h"
#include "common/utils.h"
#include "common/vector.h"
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
 * For each alignment condition (ALIGN_CW0, CW90, etc) the source vector under
 * test is set to a unit vector along each axis (x-axis, y-axis, z-axis)
 * plus one additional random vector is tested.
 */

#define DEG2RAD 0.01745329251

static void rotateVector(int32_t mat[3][3], vector3_t *vec, vector3_t *out)
{
    vector3_t tmp;

    for(int i = 0; i < 3; i++) {
        tmp.v[i] = 0;
        for(int j = 0; j < 3; j++) {
            tmp.v[i] += mat[j][i] * vec->v[j];
        }
    }

    *out = tmp;
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

static void alignSensorViaMatrixFromRotation(vector3_t *dest, sensor_align_e alignment)
{
    matrix33_t sensorRotationMatrix;

    sensorAlignment_t sensorAlignment;

    buildAlignmentFromStandardAlignment(&sensorAlignment, alignment);

    buildRotationMatrixFromAngles(&sensorRotationMatrix, &sensorAlignment);

    alignSensorViaMatrix(dest, &sensorRotationMatrix);
}

static void testCW(sensor_align_e rotation, int32_t angle)
{
    vector3_t src;
    vector3_t test;

    // unit vector along x-axis
    src.x = 1;
    src.y = 0;
    src.z = 0;

    int32_t matrix[3][3];
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, &src, &test);

    alignSensorViaMatrixFromRotation(&src, rotation);
    EXPECT_NEAR(test.x, src.x, TOL) << "X-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "X-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "X-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

    // unit vector along y-axis
    src.x = 0;
    src.y = 1;
    src.z = 0;

    rotateVector(matrix, &src, &test);
    alignSensorViaMatrixFromRotation(&src, rotation);
    EXPECT_NEAR(test.x, src.x, TOL) << "Y-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Y-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Y-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

    // unit vector along z-axis
    src.x = 0;
    src.y = 0;
    src.z = 1;

    rotateVector(matrix, &src, &test);
    alignSensorViaMatrixFromRotation(&src, rotation);
    EXPECT_NEAR(test.x, src.x, TOL) << "Z-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Z-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Z-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

    // random vector to test
    src.x = rand() % 5;
    src.y = rand() % 5;
    src.z = rand() % 5;

    rotateVector(matrix, &src, &test);
    alignSensorViaMatrixFromRotation(&src,  rotation);
    EXPECT_NEAR(test.x, src.x, TOL) << "Random alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Random alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Random alignment does not match in Z-Axis. " << test.z << " " << src.z;
}

/*
 * Since the order of flip and rotation matters, these tests make the
 * assumption that the 'flip' occurs first, followed by clockwise rotation
 */
static void testCWFlip(sensor_align_e rotation, int32_t angle)
{
    vector3_t src;
    vector3_t test;

    // unit vector along x-axis
    src.x = 1;
    src.y = 0;
    src.z = 0;

    int32_t matrix[3][3];
    initYAxisRotation(matrix, 180);
    rotateVector(matrix, &src, &test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, &test, &test);

    alignSensorViaMatrixFromRotation(&src, rotation);

    EXPECT_NEAR(test.x, src.x, TOL) << "X-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "X-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "X-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

    // unit vector along y-axis
    src.x = 0;
    src.y = 1;
    src.z = 0;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, &src, &test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, &test, &test);

    alignSensorViaMatrixFromRotation(&src, rotation);

    EXPECT_NEAR(test.x, src.x, TOL) << "Y-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Y-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Y-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

    // unit vector along z-axis
    src.x = 0;
    src.y = 0;
    src.z = 1;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, &src, &test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, &test, &test);

    alignSensorViaMatrixFromRotation(&src, rotation);

    EXPECT_NEAR(test.x, src.x, TOL) << "Z-Unit alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Z-Unit alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Z-Unit alignment does not match in Z-Axis. " << test.z << " " << src.z;

     // random vector to test
    src.x = rand() % 5;
    src.y = rand() % 5;
    src.z = rand() % 5;

    initYAxisRotation(matrix, 180);
    rotateVector(matrix, &src, &test);
    initZAxisRotation(matrix, angle);
    rotateVector(matrix, &test, &test);

    alignSensorViaMatrixFromRotation(&src, rotation);

    EXPECT_NEAR(test.x, src.x, TOL) << "Random alignment does not match in X-Axis. " << test.x << " " << src.x;
    EXPECT_NEAR(test.y, src.y, TOL) << "Random alignment does not match in Y-Axis. " << test.y << " " << src.y;
    EXPECT_NEAR(test.z, src.z, TOL) << "Random alignment does not match in Z-Axis. " << test.z << " " << src.z;
}


TEST(AlignSensorTest, ClockwiseZeroDegrees)
{
    srand(time(NULL));
    testCW(CW0_DEG, 0);
}

TEST(AlignSensorTest, ClockwiseNinetyDegrees)
{
    testCW(CW90_DEG, 90);
}

TEST(AlignSensorTest, ClockwiseOneEightyDegrees)
{
    testCW(CW180_DEG, 180);
}

TEST(AlignSensorTest, ClockwiseTwoSeventyDegrees)
{
    testCW(CW270_DEG, 270);
}

TEST(AlignSensorTest, ClockwiseZeroDegreesFlip)
{
    testCWFlip(CW0_DEG_FLIP, 0);
}

TEST(AlignSensorTest, ClockwiseNinetyDegreesFlip)
{
    testCWFlip(CW90_DEG_FLIP, 90);
}

TEST(AlignSensorTest, ClockwiseOneEightyDegreesFlip)
{
    testCWFlip(CW180_DEG_FLIP, 180);
}

TEST(AlignSensorTest, ClockwiseTwoSeventyDegreesFlip)
{
    testCWFlip(CW270_DEG_FLIP, 270);
}

static void testBuildAlignmentWithStandardAlignment(sensor_align_e alignment, sensorAlignment_t expectedSensorAlignment)
{
    sensorAlignment_t sensorAlignment = SENSOR_ALIGNMENT(6, 6, 6);

    buildAlignmentFromStandardAlignment(&sensorAlignment, alignment);

    for (unsigned i = 0; i < ARRAYLEN(sensorAlignment.raw); i++) {
        EXPECT_EQ(expectedSensorAlignment.raw[i], sensorAlignment.raw[i]) << "Sensor alignment was not updated. alignment: " << alignment;
    }
}

TEST(AlignSensorTest, AttemptBuildAlignmentWithStandardAlignment)
{
    testBuildAlignmentWithStandardAlignment(CW0_DEG,           CUSTOM_ALIGN_CW0_DEG);
    testBuildAlignmentWithStandardAlignment(CW90_DEG,          CUSTOM_ALIGN_CW90_DEG);
    testBuildAlignmentWithStandardAlignment(CW180_DEG,         CUSTOM_ALIGN_CW180_DEG);
    testBuildAlignmentWithStandardAlignment(CW270_DEG,         CUSTOM_ALIGN_CW270_DEG);
    testBuildAlignmentWithStandardAlignment(CW0_DEG_FLIP,      CUSTOM_ALIGN_CW0_DEG_FLIP);
    testBuildAlignmentWithStandardAlignment(CW90_DEG_FLIP,     CUSTOM_ALIGN_CW90_DEG_FLIP);
    testBuildAlignmentWithStandardAlignment(CW180_DEG_FLIP,    CUSTOM_ALIGN_CW180_DEG_FLIP);
    testBuildAlignmentWithStandardAlignment(CW270_DEG_FLIP,    CUSTOM_ALIGN_CW270_DEG_FLIP);
}

TEST(AlignSensorTest, AttemptBuildAlignmentFromCustomAlignment)
{
    sensorAlignment_t sensorAlignment = SENSOR_ALIGNMENT(1, 2, 3);

    buildAlignmentFromStandardAlignment(&sensorAlignment, ALIGN_CUSTOM);

    sensorAlignment_t expectedSensorAlignment = SENSOR_ALIGNMENT(1, 2, 3);

    for (unsigned i = 0; i < ARRAYLEN(sensorAlignment.raw); i++) {
        EXPECT_EQ(expectedSensorAlignment.raw[i], sensorAlignment.raw[i]) << "Custom alignment should not be updated.";
    }
}

TEST(AlignSensorTest, AttemptBuildAlignmentFromDefaultAlignment)
{
    sensorAlignment_t sensorAlignment = SENSOR_ALIGNMENT(1, 2, 3);

    buildAlignmentFromStandardAlignment(&sensorAlignment, ALIGN_DEFAULT);

    sensorAlignment_t expectedSensorAlignment = SENSOR_ALIGNMENT(1, 2, 3);

    for (unsigned i = 0; i < ARRAYLEN(sensorAlignment.raw); i++) {
        EXPECT_EQ(expectedSensorAlignment.raw[i], sensorAlignment.raw[i]) << "Default alignment should not be updated.";
    }
}

TEST(AlignSensorTest, AlignmentBitmasks)
{
    uint8_t bits;

    bits = ALIGNMENT_TO_BITMASK(CW0_DEG);
    EXPECT_EQ(0x0, bits); // 000000
    EXPECT_EQ(0, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW90_DEG);
    EXPECT_EQ(0x1, bits); // 000001
    EXPECT_EQ(1, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(1, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW180_DEG);
    EXPECT_EQ(0x2, bits); // 000010
    EXPECT_EQ(2, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW270_DEG);
    EXPECT_EQ(0x3, bits); // 000011
    EXPECT_EQ(3, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(3, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW0_DEG_FLIP);
    EXPECT_EQ(0x8, bits); // 001000
    EXPECT_EQ(0, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(2, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW90_DEG_FLIP);
    EXPECT_EQ(0x9, bits); // 001001
    EXPECT_EQ(1, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(2, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(1, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW180_DEG_FLIP);
    EXPECT_EQ(0xA, bits); // 001010
    EXPECT_EQ(2, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(2, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));

    bits = ALIGNMENT_TO_BITMASK(CW270_DEG_FLIP);
    EXPECT_EQ(0xB, bits); // 001011
    EXPECT_EQ(3, ALIGNMENT_YAW_ROTATIONS(bits));
    EXPECT_EQ(2, ALIGNMENT_PITCH_ROTATIONS(bits));
    EXPECT_EQ(0, ALIGNMENT_ROLL_ROTATIONS(bits));

    EXPECT_EQ(3, ALIGNMENT_AXIS_ROTATIONS(bits, FD_YAW));
    EXPECT_EQ(2, ALIGNMENT_AXIS_ROTATIONS(bits, FD_PITCH));
    EXPECT_EQ(0, ALIGNMENT_AXIS_ROTATIONS(bits, FD_ROLL));
}
