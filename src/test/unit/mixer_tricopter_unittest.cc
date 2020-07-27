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

extern "C" {
#include "common/maths.h"
#include "build/debug.h"
#include "platform.h"

#include "fc/runtime_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "drivers/sensor.h"

#include "flight/mixer.h"
#include "flight/mixer_tricopter.h"

#include "io/beeper.h"
#include "fc/rc_controls.h"
#include "rx/rx.h"

int16_t servo[MAX_SUPPORTED_SERVOS];
int16_t test_motorLow;
int16_t test_motorHigh;
int16_t test_motorRange;

void tailTuneModeThrustTorque(thrustTorque_t *pTT, const bool isThrottleHigh);
uint16_t getLinearServoValue(servoParam_t *pServo, float scaledPIDOutput, float pidSumLimit);
float getAngleForYawOutput(float yawOutput);
uint16_t getServoValueAtAngle(servoParam_t *pServo, float angle);
float getServoAngle(servoParam_t *pServo, uint16_t servoValue);

} // extern "C"

extern tailServo_t tailServo;
extern tailMotor_t tailMotor;
extern tailTune_t tailTune;

#include "unittest_macros.h"
#include "gtest/gtest.h"

class ThrustFactorCalculationTest: public ::testing::Test {
// We expect factor = 1 / tan(angle) (but adjusted for formats)
// Say we want triflightConfig()->tri_tail_motor_thrustfactor to be 139, i.e. the factor should be 13.9
// angle = 1 / atan(factor), according to #25
// adjust to decidegrees and multiply by servoAvgAngle.numOf
// i.e. multiply by 3000, then round to integer
// so even if 12345 looks like an arbitrarily chosen number, it is the result of this calculation and corresponds to 4.115 degrees.
// after that, add 270000 (90 deg) since the angles actually start at horizontal left
// Due to possible rounding effects we add a tolerance to the test
  protected:
    virtual void SetUp() {
        PG_RESET(triflightConfig);
        memset(&tailServo, 0, sizeof(tailServo));
        tailServo.pConf = static_cast<servoParam_t *>(malloc(sizeof(*(tailServo.pConf))));
        memset(tailServo.pConf, 0, sizeof(*(tailServo.pConf)));
        tailServo.pConf->min = DEFAULT_SERVO_MIN;
        tailServo.pConf->max = DEFAULT_SERVO_MAX;
        tailServo.pConf->middle = DEFAULT_SERVO_MIDDLE;
        tailServo.pConf->rate = 100;
        tailServo.pConf->forwardFromChannel = CHANNEL_FORWARDING_DISABLED;

        // give all servos a default command
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo[i] = DEFAULT_SERVO_MIDDLE;
        }

        triflightConfigMutable()->tri_tail_motor_thrustfactor = 123; // so we can check it's unchanged on TT_FAIL
        triInitMixer(tailServo.pConf, &servo[5]);
        tailTune.mode = TT_MODE_THRUST_TORQUE;
        tailTune.tt.state = TT_WAIT_FOR_DISARM;
        tailTune.tt.servoAvgAngle.numOf = 300;
    }

    virtual void TearDown() {
      free(tailServo.pConf);
    }
};

TEST_F(ThrustFactorCalculationTest, 139) {
    // given
    tailTune.tt.servoAvgAngle.sum = 1234.5 + 27000.0;
    EXPECT_EQ(1,1);
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(139, triflightConfig()->tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 145) {
    // given
    tailTune.tt.servoAvgAngle.sum = 1183.6 + 27000.0;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(145, triflightConfig()->tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 125) {
    // given
    tailTune.tt.servoAvgAngle.sum = 1372.2 + 27000.0;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(125, triflightConfig()->tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, 80) {
    // given
    tailTune.tt.servoAvgAngle.sum = 2137.5 + 27000.0;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_NEAR(80, triflightConfig()->tri_tail_motor_thrustfactor, 1);
    EXPECT_EQ(tailTune.tt.state, TT_DONE);
}

TEST_F(ThrustFactorCalculationTest, err90) {
    // given
    tailTune.tt.servoAvgAngle.sum = 27000.0;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_EQ(123, triflightConfig()->tri_tail_motor_thrustfactor);
    EXPECT_EQ(tailTune.tt.state, TT_FAIL);
}

TEST_F(ThrustFactorCalculationTest, err130) {
    // given
    tailTune.tt.servoAvgAngle.sum = 39000.0;
    // and
    tailTuneModeThrustTorque(&tailTune.tt, true);
    // then
    EXPECT_EQ(123, triflightConfig()->tri_tail_motor_thrustfactor);
    EXPECT_EQ(tailTune.tt.state, TT_FAIL);
}

class LinearOutputTest: public ::testing::Test {
  protected:
    virtual void SetUp() {

        test_motorLow = 48;
        test_motorHigh = 2000;
        test_motorRange = test_motorHigh - test_motorLow;
        memset(&tailServo, 0, sizeof(tailServo));
        tailServo.pConf = static_cast<servoParam_t *>(malloc(sizeof(*(tailServo.pConf))));
        memset(tailServo.pConf, 0, sizeof(*(tailServo.pConf)));
        tailServo.pConf->min = DEFAULT_SERVO_MIN;
        tailServo.pConf->max = DEFAULT_SERVO_MAX;
        tailServo.pConf->middle = DEFAULT_SERVO_MIDDLE;
        tailServo.pConf->rate = 100;
        tailServo.pConf->forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
        tailServo.angleAtMax = 40;

        // give all servos a default command
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servo[i] = DEFAULT_SERVO_MIDDLE;
        }

        triflightConfigMutable()->tri_tail_motor_thrustfactor = 54;
        triflightConfigMutable()->tri_yaw_boost = 240;
        triInitMixer(tailServo.pConf, &servo[5]);
        tailTune.mode = TT_MODE_THRUST_TORQUE;
        tailTune.tt.state = TT_WAIT_FOR_DISARM;
        tailTune.tt.servoAvgAngle.numOf = 300;
    }

    virtual void TearDown(){
      free(tailServo.pConf);
    }

    virtual float getYaw0Angle(float thrustFactor) {
        float angle = 0;
        float smallestOutput = 100000;
        for (angle = 40; angle < 180.0; angle += 0.01) {
            float angleRad = DEGREES_TO_RADIANS(angle);
            float output = (-thrustFactor * cos(angleRad) - sin(angleRad));
            if (fabsf(output) < smallestOutput) {
                smallestOutput = fabsf(output);
            } else {
                break;
            }
        }

        return angle;
    }
};

TEST_F(LinearOutputTest, getAngleForYawOutput_motor0_output0Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0;
    float output = 0;
    float angle = this->getYaw0Angle(triflightConfig()->tri_tail_motor_thrustfactor / 10.0);
    EXPECT_NEAR(angle, getAngleForYawOutput(output), 0.05);
    EXPECT_NEAR(71, tailServo.angleAtLinearMin, 0.05);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_motor0_output50Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0;
    float output = tailServo.maxYawOutput * 0.5;
    EXPECT_NEAR(122.8, getAngleForYawOutput(output), 1);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_motor0_output100Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0;
    float output = tailServo.maxYawOutput * 1;
    EXPECT_NEAR(tailServo.angleAtLinearMax, getAngleForYawOutput(output), 2);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_motor0_outputNeg100Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0;
    float output = -tailServo.maxYawOutput * 1;
    EXPECT_NEAR(tailServo.angleAtLinearMin, getAngleForYawOutput(output), 2);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_motor0_outputNeg50Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0;
    float output = -tailServo.maxYawOutput * 0.5;
    EXPECT_NEAR(78.1, getAngleForYawOutput(output), 1);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_motor40_output50Percent) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0.4;
    float output = tailServo.maxYawOutput * 0.5;
    EXPECT_NEAR(111.5, getAngleForYawOutput(output), 1);
}

TEST_F(LinearOutputTest, getAngleForYawOutput_accuracy) {
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0.5;
    float output = tailServo.maxYawOutput * 0.5;
    const float angle = getAngleForYawOutput(output);
    output *= 1.001;
    const float secondAngle = getAngleForYawOutput(output);
    EXPECT_NEAR(0.01, fabsf(secondAngle - angle), 0.005);
}

TEST_F(LinearOutputTest, getServoValueAtAngle_min) {
    uint16_t angle = tailServo.angleAtMin;
    EXPECT_EQ(tailServo.pConf->min, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_mid) {
    uint16_t angle = TRI_TAIL_SERVO_ANGLE_MID;
    EXPECT_EQ(tailServo.pConf->middle, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_max) {
    float angle = tailServo.angleAtMax;
    EXPECT_EQ(tailServo.pConf->max, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_1percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID + (tailServo.angleAtMax - TRI_TAIL_SERVO_ANGLE_MID) * 0.01;
    EXPECT_EQ(tailServo.pConf->middle + (tailServo.pConf->max - tailServo.pConf->middle) * 0.01, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_neg1percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID - (TRI_TAIL_SERVO_ANGLE_MID - tailServo.angleAtMin) * 0.01;
    EXPECT_NEAR(tailServo.pConf->middle - (tailServo.pConf->middle - tailServo.pConf->min) * 0.01, getServoValueAtAngle(tailServo.pConf, angle), 1);
}

TEST_F(LinearOutputTest, getServoValueAtAngle_50percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID + (tailServo.angleAtMax - TRI_TAIL_SERVO_ANGLE_MID) * 0.5;
    EXPECT_EQ(tailServo.pConf->middle + (tailServo.pConf->max - tailServo.pConf->middle) * 0.5, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_neg50percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID - (TRI_TAIL_SERVO_ANGLE_MID - tailServo.angleAtMin) * 0.5;
    EXPECT_NEAR(tailServo.pConf->middle - (tailServo.pConf->middle - tailServo.pConf->min) * 0.5, getServoValueAtAngle(tailServo.pConf, angle), 1);
}

TEST_F(LinearOutputTest, getServoValueAtAngle_110percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID + (tailServo.angleAtMax - TRI_TAIL_SERVO_ANGLE_MID) * 1.1;
    EXPECT_EQ(tailServo.pConf->middle + (tailServo.pConf->max - tailServo.pConf->middle) * 1.1, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoValueAtAngle_neg110percent) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID - (TRI_TAIL_SERVO_ANGLE_MID - tailServo.angleAtMin) * 1.1;
    EXPECT_NEAR(tailServo.pConf->middle - (tailServo.pConf->middle - tailServo.pConf->min) * 1.1, getServoValueAtAngle(tailServo.pConf, angle), 1);
}

TEST_F(LinearOutputTest, getServoValueAtAngle_resolution) {
    float angle = TRI_TAIL_SERVO_ANGLE_MID;
    int16_t value = getServoValueAtAngle(tailServo.pConf, angle);
    angle += 0.1f;
    EXPECT_NE(value, getServoValueAtAngle(tailServo.pConf, angle));
    angle -= 0.2f;
    EXPECT_NE(value, getServoValueAtAngle(tailServo.pConf, angle));
}

TEST_F(LinearOutputTest, getServoAngle_min) {
    uint16_t servoValue = tailServo.pConf->min;
    EXPECT_NEAR(tailServo.angleAtMin, getServoAngle(tailServo.pConf, servoValue), 1);
}

TEST_F(LinearOutputTest, getServoAngle_max) {
    uint16_t servoValue = tailServo.pConf->max;
    EXPECT_NEAR(tailServo.angleAtMax, getServoAngle(tailServo.pConf, servoValue), 1);
}

TEST_F(LinearOutputTest, getServoAngle_mid) {
    uint16_t servoValue = tailServo.pConf->middle;
    EXPECT_NEAR(TRI_TAIL_SERVO_ANGLE_MID, getServoAngle(tailServo.pConf, servoValue), 1);
}

TEST_F(LinearOutputTest, getServoAngle_50percent) {
    uint16_t servoValue = tailServo.pConf->middle + (tailServo.pConf->max - tailServo.pConf->middle) * 0.5;
    EXPECT_NEAR(TRI_TAIL_SERVO_ANGLE_MID + (tailServo.angleAtMax - TRI_TAIL_SERVO_ANGLE_MID) * 0.5, getServoAngle(tailServo.pConf, servoValue), 1);
}

TEST_F(LinearOutputTest, getServoAngle_neg50percent) {
    uint16_t servoValue = tailServo.pConf->middle - (tailServo.pConf->middle - tailServo.pConf->min) * 0.5;
    EXPECT_NEAR(TRI_TAIL_SERVO_ANGLE_MID - (TRI_TAIL_SERVO_ANGLE_MID - tailServo.angleAtMin) * 0.5, getServoAngle(tailServo.pConf, servoValue), 1);
}

TEST_F(LinearOutputTest, getLinearServoValue_accuracy) {
    float pidOutput = 500;
    const float pidLimit = 1000;
    tailMotor.virtualFeedBack = test_motorLow + test_motorRange * 0.5f;
    const uint16_t servoValue = getLinearServoValue(tailServo.pConf, pidOutput, pidLimit);
    pidOutput += 3.0f;
    const uint16_t newServoValue = getLinearServoValue(tailServo.pConf, pidOutput, pidLimit);
    EXPECT_EQ(1, newServoValue - servoValue);
}

//STUBS
extern "C" {

//typedef struct master_s {
//} master_t;

float dT;
uint8_t armingFlags;
float rcCommand[4];
uint32_t rcModeActivationMask;
uint16_t flightModeFlags = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
bool airModeActive = true;
uint8_t debugMode = DEBUG_TRIFLIGHT;

uint32_t millis(void) {
    return 0;
}

uint16_t getCurrentMinthrottle(void) {
    return 0;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

uint16_t disableFlightMode(flightModeFlags_e mask) {
    UNUSED(mask);
    return 0;
}

void beeperConfirmationBeeps(uint8_t beepCount) {
    UNUSED(beepCount);
}

uint16_t enableFlightMode(flightModeFlags_e mask) {
    UNUSED(mask);
    return 0;
}

throttleStatus_e calculateThrottleStatus() {
    return (throttleStatus_e) 0;
}

uint16_t adcGetChannel(uint8_t channel) {
    UNUSED(channel);
    return 0;
}

float pt1FilterApply(pt1Filter_t *filter, float input) {
    UNUSED(filter);
    UNUSED(input);
    return 0.0;
}

void saveConfigAndNotify(void) {
}

void pidSetPredictedError(flight_dynamics_index_t axis, float error) {
    UNUSED(axis);
    UNUSED(error);
}

void pt1FilterInit(pt1Filter_t *filter, float k)
{
    UNUSED(filter);
    UNUSED(k);
}

bool isAirmodeActivated(void)
{
    return airModeActive;
}

int servoDirection(int servoIndex, int inputSource)
{
    UNUSED(servoIndex);
    UNUSED(inputSource);
    return 1;
}

uint16_t mixGetMotorOutputLow()
{
    return test_motorLow;
}

uint16_t mixGetMotorOutputHigh()
{
    return test_motorHigh;
}

void triInitFilters(void){

}

} // extern "C"
