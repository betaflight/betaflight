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

#include "unittest_macros.h"
#include "gtest/gtest.h"
#include "platform.h"

extern "C" {
    #include "common/axis.h"
    #include "common/maths.h"
    #include "flight/pid.h"
    #include "flight/psas.h"
    #include "flight/psas_trimming.h"
    #include "build/debug.h"
    #include "sensors/gyro.h"
    #include "sensors/acceleration.h"
    #include "io/gps.h"
    #include "fc/rc_modes.h"

    float updateAccelZHoldingController(float pitchStick, float accelZ);

    pidRuntime_t pidRuntime;
    extern psasRuntime_t psasRuntime;
    extern psas_data_t psasData;
    extern psas_trimming_data_t psasTrimmingData;
    pidAxisData_t pidData[XYZ_AXIS_COUNT];
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    acc_t acc;
    gyro_t gyro;
    gpsSolutionData_t gpsSol;
    uint8_t stateFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t armingFlags = 0;
    float testRcDeflection[3] = { 0.0f, 0.0f, 0.0f };
    float testMaxRate[3] = { 670,670,670 };

    bool isFixedWing(void)
    {
        return true;
    }
    float getMaxRcRate(int axis)
    {
        float maxRate = testMaxRate[axis];
        return maxRate;
    }
    float getSetpointRate(int axis)
    {
        UNUSED(axis);
        return 10.0f;
    }
    float getRcDeflection(int axis) { return testRcDeflection[axis]; }
    float getTpaFactor(const pidProfile_t *pidProfile, int axis, term_e term)
    {
        UNUSED(pidProfile);
        UNUSED(axis);
        UNUSED(term);
        return 1.0f;
    }
    bool IS_RC_MODE_ACTIVE(boxId_e boxId)
    {
        UNUSED(boxId);
        return false;
    }
    bool sensors(uint32_t mask) { return (mask == SENSOR_ACC) || (mask == SENSOR_PITOT); }
    bool gyroOverflowDetected(void) { return false; }
}

static void resetPsasState(void)
{
    memset(&psasData,    0, sizeof(psasData));
    memset(&psasRuntime, 0, sizeof(psasRuntime));

    psasRuntime.pitch_accel_max    = 2.0f;   // +2G
    psasRuntime.pitch_accel_min    = -1.0f;  // -1G
    psasRuntime.pitch_accel_p_gain = 50.0f;
    psasRuntime.pitch_accel_i_gain = 100.0f;
    psasRuntime.servoVelocityLimit = 100.0f;
    psasRuntime.speed_gains.stick[FD_PITCH] = 1.0f;

    pidRuntime.dT = 0.001f; // 1 kHz
}

TEST(PsasAccelZHold, ZeroErrorProducesZeroOutput)
{
    resetPsasState();
    // stick=0 → req=1.0; accelZ=1.0 → delta=0
    const float out = updateAccelZHoldingController(0.0f, 1.0f);
    EXPECT_FLOAT_EQ(out, 0.0f);
    EXPECT_FLOAT_EQ(psasData.pitch.I, 0.0f);
}

TEST(PsasAccelZHold, PositiveErrorProducesPositiveOutput)
{
    resetPsasState();
    // delta=+1.0; P=1.0*50=50; I+=100*0.001=0.1
    const float out = updateAccelZHoldingController(0.0f, 2.0f);
    EXPECT_FLOAT_EQ(out, 50.0f);
    EXPECT_FLOAT_EQ(psasData.pitch.I, 0.1f);
}

TEST(PsasAccelZHold, NegativeErrorProducesNegativeOutput)
{
    resetPsasState();
    // delta=-0.5; P=-25; I=-0.05
    const float out = updateAccelZHoldingController(0.0f, 0.5f);
    EXPECT_FLOAT_EQ(out, -25.0f);
    EXPECT_FLOAT_EQ(psasData.pitch.I, -0.05f);
}

TEST(PsasAccelZHold, FullNegativeStickRequestsMaxAccel)
{
    resetPsasState();
    // stick=-1 → req=2.0 (максимум); accelZ=2.0 → delta=0
    const float outMax = updateAccelZHoldingController(-1.0f, 2.0f);
    EXPECT_FLOAT_EQ(outMax, 0.0f);

    // stick=+1 → req=-1.0 (минимум/клэмп); accelZ=-1.0 → delta=0
    const float outMin = updateAccelZHoldingController(1.0f, -1.0f);
    EXPECT_NEAR(outMin, 0.0f, 1e-5f);
}

TEST(PsasAccelZHold, FullPositiveStickRequestsMinAccel)
{
    resetPsasState();
    // stick=+1 → req=-1.0; accelZ=-1.0 → delta=0
    const float out = updateAccelZHoldingController(1.0f, -1.0f);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST(PsasAccelZHold, HalfPositiveStickRequestsZeroG)
{
    resetPsasState();
    // stick=+0.5 → req=0.0; accelZ=0.0 → delta=0
    const float out = updateAccelZHoldingController(0.5f, 0.0f);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST(PsasAccelZHold, ItermAccumulatesOverIterations)
{
    resetPsasState();
    // 10 iterations × 0.1 → I=1.0
    for (int i = 0; i < 10; i++) {
        updateAccelZHoldingController(0.0f, 2.0f);
    }
    EXPECT_NEAR(psasData.pitch.I, 1.0f, 1e-5f);
}

TEST(PsasAccelZHold, PositiveServoVelocityIsClamped)
{
    resetPsasState();
    psasRuntime.pitch_accel_i_gain = 100000.0f;
    // delta=1.0; velocity=100000 → clamp 100; I += 100*0.001 = 0.1
    updateAccelZHoldingController(0.0f, 2.0f);
    EXPECT_NEAR(psasData.pitch.I, 0.1f, 1e-5f);
}

TEST(PsasAccelZHold, NegativeServoVelocityIsClamped)
{
    resetPsasState();
    psasRuntime.pitch_accel_i_gain = 100000.0f;
    // delta=-1.0 → velocity=-100000 → clamp -100; I=-0.1
    updateAccelZHoldingController(0.0f, 0.0f);
    EXPECT_NEAR(psasData.pitch.I, -0.1f, 1e-5f);
}

TEST(PsasAccelZHold, SymmetricResponse)
{
    resetPsasState();
    const float outPos   = updateAccelZHoldingController(0.0f, 2.0f); // delta=+1
    const float itermPos = psasData.pitch.I;

    resetPsasState();
    const float outNeg   = updateAccelZHoldingController(0.0f, 0.0f); // delta=-1
    const float itermNeg = psasData.pitch.I;

    EXPECT_NEAR(outPos,   -outNeg,   1e-5f);
    EXPECT_NEAR(itermPos, -itermNeg, 1e-5f);
}

TEST(PsasAccelZHold, SpeedGainCompressesAccelRange)
{
    resetPsasState();
    psasRuntime.pitch_accel_max = 3.0f;
    psasRuntime.pitch_accel_min = -2.0f;
    psasRuntime.speed_gains.stick[FD_PITCH] = 1.5f;
    // max=MAX(3/1.5,2)=2.0; min=MIN(-2/1.5,-1)=-1.333
    // stick=-1 → req=(1-2)*(-1)+1=2.0; accelZ=2.0 → delta=0
    const float outMax = updateAccelZHoldingController(-1.0f, 2.0f);
    EXPECT_FLOAT_EQ(outMax, 0.0f);

    const float outMin = updateAccelZHoldingController(1.0f, -2.0f / 1.5f);
    EXPECT_NEAR(outMin, 0.0f, 1e-5f);
}

static void setupTrimming(unsigned mask, float cli_deadband, float cli_rate,
                          float cli_limit, float cli_tau_reset)
{
    pidRuntime.dT = 0.001f;   // 1 kHz

    memset(&psasTrimmingData, 0, sizeof(psasTrimmingData));

    // k=1 → pt1FilterApply returned input
    for (uint8_t axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterInit(&psasTrimmingData.stickLowpass[axis], 1.0f);
    }

    psasTrimmingData.enabledMask = mask;
    psasTrimmingData.deadband    = cli_deadband * 0.1f;
    psasTrimmingData.rate        = cli_rate     * 0.01f;
    psasTrimmingData.outputRange = cli_limit;
    psasTrimmingData.tauReset    = cli_tau_reset * 0.001f;
    if (mask == 0) {
        psasTrimmingData.state = TRIMMING_OFF;
    }

    memset(testRcDeflection, 0, sizeof(testRcDeflection));
}

TEST(PsasTrimming, InitZeroMaskTurnsOffState)
{
    // Prepare state with nonzero outputs and active TRIMMING_ADJUSTMENT.
    // setupTrimming(0x07,...) leaves enabledMask = 0x07 and does NOT call memset
    // after we set output/state — so the outputs survive until psasTrimmingInit() is called.
    setupTrimming(0x07, 1, 100, 50, 100);

    psasTrimmingData.state = TRIMMING_ADJUSTMENT;
    psasTrimmingData.activeAxis = FD_ROLL;
    psasTrimmingData.output[FD_ROLL]  =  1.0f;
    psasTrimmingData.output[FD_PITCH] = -2.0f;
    psasTrimmingData.output[FD_YAW]   =  3.0f;

    // Profile with fully disabled trimming mask.
    // lowpass_time = 0 → pt1FilterGainFromDelay(0, dT) yields k=1,
    // which is safe for subsequent psasTrimmingUpdate() calls.
    pidProfile_t profile = {};
    profile.psas_trim_enabled_mask = 0;
    profile.psas_trim_lowpass_time = 0;
    profile.psas_trim_deadband    = 0;
    profile.psas_trim_rate        = 0;
    profile.psas_trim_limit       = 0;
    profile.psas_trim_tau_reset   = 0;

    psasTrimmingInit(&profile);

    // Verify that psasTrimmingInit() itself zeroed the outputs
    // and forced the state to TRIMMING_OFF.
    EXPECT_EQ(getPsasTrimmingState(), TRIMMING_OFF);
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_ROLL),  0.0f);
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_PITCH), 0.0f);
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_YAW),   0.0f);
}

TEST(PsasTrimming, InitDisablesOnlyDisabledAxes)
{
    // Additional coverage: mask 0x05 (ROLL|YAW) — PITCH must be zeroed,
    // while ROLL/YAW are preserved (clamped to outputRange).
    setupTrimming(0x07, 1, 100, 10, 100);
    psasTrimmingData.output[FD_ROLL]  =  5.0f;
    psasTrimmingData.output[FD_PITCH] =  7.0f;
    psasTrimmingData.output[FD_YAW]   = 15.0f; // > limit → must be clamped

    pidProfile_t profile = {};
    profile.psas_trim_enabled_mask = 0x05;
    profile.psas_trim_lowpass_time = 0;
    profile.psas_trim_limit        = 10;

    psasTrimmingInit(&profile);

    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_ROLL),   5.0f);
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_PITCH),  0.0f);
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_YAW),   10.0f); // clamped
}

TEST(PsasTrimming, SetStateIgnoredWithZeroMask)
{
    setupTrimming(0, 0, 0, 0, 100);
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);
    EXPECT_EQ(getPsasTrimmingState(), TRIMMING_OFF);
}

TEST(PsasTrimming, AdjustmentPicksMaxDeflectionAxis)
{
    setupTrimming(0x07, 1, 100, 50, 100);
    testRcDeflection[FD_ROLL]  =  0.1f;
    testRcDeflection[FD_PITCH] = -0.5f;   // |-50| = 50 → max
    testRcDeflection[FD_YAW]   =  0.2f;

    setPsasTrimmingState(TRIMMING_ADJUSTMENT);

    EXPECT_EQ(getPsasTrimmingState(), TRIMMING_ADJUSTMENT);
    EXPECT_EQ(getPsasTrimmingActiveChannel(), FD_PITCH);
}

TEST(PsasTrimming, AdjustmentIgnoresDisabledAxes)
{
    setupTrimming((1 << FD_ROLL) | (1 << FD_YAW), 1, 100, 50, 100);
    testRcDeflection[FD_ROLL]  = 0.1f;
    testRcDeflection[FD_PITCH] = 0.9f;
    testRcDeflection[FD_YAW]   = 0.3f;

    setPsasTrimmingState(TRIMMING_ADJUSTMENT);

    EXPECT_EQ(getPsasTrimmingActiveChannel(), FD_YAW);
}

TEST(PsasTrimming, AdjustmentAccumulatesOutputWithStick)
{
    // filter k=1 → stickFiltered = deflection * 100 = 50
    // output += 50 * rate(1.0) * dT(0.001) = 0.05 by one step
    setupTrimming(0x07, 1, 100, 50, 100);
    testRcDeflection[FD_ROLL] = 0.5f;
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);

    psasTrimmingUpdate();
    EXPECT_NEAR(getPsasTrimmingOutput(FD_ROLL), 0.05f, 1e-5f);

    psasTrimmingUpdate();
    EXPECT_NEAR(getPsasTrimmingOutput(FD_ROLL), 0.10f, 1e-5f);

    for (int i = 0; i < 8; i++) psasTrimmingUpdate();
    EXPECT_NEAR(getPsasTrimmingOutput(FD_ROLL), 0.50f, 1e-4f);
}

TEST(PsasTrimming, AdjustmentRespectsDeadband)
{
    // deadband = 200 * 0.1 = 20; stick = 10 < 20 → no integral
    setupTrimming(0x07, 200, 100, 50, 100);
    testRcDeflection[FD_ROLL] = 0.1f;
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);

    for (int i = 0; i < 100; i++) psasTrimmingUpdate();
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_ROLL), 0.0f);
}

TEST(PsasTrimming, AdjustmentClampsToOutputRange)
{
    // rate = 10000 * 0.01 = 100; за шаг 100 * 100 * 0.001 = 10
    // limit = 5 → clamped
    setupTrimming(0x07, 1, 10000, 5, 100);
    testRcDeflection[FD_ROLL] = 1.0f;
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);

    for (int i = 0; i < 10; i++) psasTrimmingUpdate();
    EXPECT_NEAR(getPsasTrimmingOutput(FD_ROLL), 5.0f, 1e-5f);
}

TEST(PsasTrimming, FixStateHoldsOutput)
{
    setupTrimming(0x07, 1, 100, 50, 100);
    testRcDeflection[FD_ROLL] = 0.5f;
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);
    for (int i = 0; i < 10; i++) psasTrimmingUpdate();
    const float accumulated = getPsasTrimmingOutput(FD_ROLL);
    ASSERT_GT(accumulated, 0.0f);

    setPsasTrimmingState(TRIMMING_FIX);
    for (int i = 0; i < 100; i++) psasTrimmingUpdate();
    EXPECT_FLOAT_EQ(getPsasTrimmingOutput(FD_ROLL), accumulated);
}

TEST(PsasTrimming, OffStateDecaysOutput)
{
    // tauReset = 100 * 0.001 = 0.1 s
    // decay: output -= output / 0.1 * 0.001 = output * 0.01  (≈ -1%/step)
    setupTrimming(0x07, 1, 100, 50, 100);
    testRcDeflection[FD_ROLL] = 0.5f;
    setPsasTrimmingState(TRIMMING_ADJUSTMENT);
    for (int i = 0; i < 10; i++) psasTrimmingUpdate();
    const float initial = getPsasTrimmingOutput(FD_ROLL);
    ASSERT_GT(initial, 0.0f);

    setPsasTrimmingState(TRIMMING_OFF);
    for (int i = 0; i < 100; i++) psasTrimmingUpdate();

    const float expected = initial * powf(0.99f, 100.0f);
    EXPECT_NEAR(getPsasTrimmingOutput(FD_ROLL), expected, 1e-4f);
    EXPECT_LT(getPsasTrimmingOutput(FD_ROLL), initial);
}
