/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <math.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/vector.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "pg/autopilot.h"

    #include "fc/rc.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "config/config.h"

    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"

    #include "flight/pid.h"

    #include "rx/rx.h"

    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);

    uint16_t flightModeFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    rxRuntimeState_t rxRuntimeState;

    gyro_t gyro;
    pidRuntime_t pidRuntime;
    uint32_t targetPidLooptime = 250;
    static timeUs_t testTimeUs;
    timeUs_t micros(void) { return testTimeUs; }

    bool isRxReceivingSignal(void) { return true; }
    bool failsafeIsActive(void) { return false; }
    bool featureIsEnabled(const uint32_t) { return false; }
    bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
    bool autopilotYawControlActive(void) { return false; }
    float autopilotGetYawRate(void) { return 0.0f; }
    void imuQuaternionHeadfreeTransformVectorEarthToBody(vector3_t *) { }

    const lowVoltageCutoff_t *getLowVoltageCutoff(void)
    {
        static const lowVoltageCutoff_t cutoff = { false, 0, 0 };
        return &cutoff;
    }

    // Not declared in rc.h: every other STATIC_UNIT_TESTED symbol in this codebase
    // (rc_adjustments.c's stepwiseAdjustments, imu.c's q/qP, etc.) is exposed to its
    // unittest the same way, since a header declaration would give every other .c
    // file that includes rc.h a stray prototype for a static-in-production symbol.
    void scaleRawSetpointToFpvCamAngle(void);
    extern float rawSetpoint[XYZ_AXIS_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// sincosf_approx() has ~3.3e-6 max error (see maths_unittest); this tolerance
// only needs to distinguish a real rotation from the identity (cos=1, sin=0)
// fallback that the pre-fix non-static locals produced, so it's set well
// above that approximation error with margin to spare.
static const float SETPOINT_TOLERANCE = 1e-2f;

static const float TEST_ROLL_INPUT = 100.0f;
static const float TEST_YAW_INPUT = 50.0f;

static void expectRawSetpointMatchesAngle(int angleDegrees)
{
    const float angleRad = DEGREES_TO_RADIANS(angleDegrees);
    const float expectedRoll = TEST_ROLL_INPUT * cosf(angleRad) - TEST_YAW_INPUT * sinf(angleRad);
    const float expectedYaw = TEST_YAW_INPUT * cosf(angleRad) + TEST_ROLL_INPUT * sinf(angleRad);

    EXPECT_NEAR(expectedRoll, rawSetpoint[ROLL], SETPOINT_TOLERANCE);
    EXPECT_NEAR(expectedYaw, rawSetpoint[YAW], SETPOINT_TOLERANCE);
}

class FpvCamAngleTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        PG_RESET(rxConfig);

        // scaleRawSetpointToFpvCamAngle() caches cos/sin factors in a function-local
        // static, so leftover state from a previous test in this binary must be
        // invalidated before each test sets its own angle. 255 is outside the
        // CLI-enforced 0-90 range (fpv_mix_degrees in cli/settings.c) but still a
        // valid uint8_t value, so it can never collide with a real test angle.
        rxConfigMutable()->fpvCamAngleDegrees = 255;
        rawSetpoint[ROLL] = 0.0f;
        rawSetpoint[PITCH] = 0.0f;
        rawSetpoint[YAW] = 0.0f;
        scaleRawSetpointToFpvCamAngle();

        rawSetpoint[ROLL] = TEST_ROLL_INPUT;
        rawSetpoint[PITCH] = 0.0f;
        rawSetpoint[YAW] = TEST_YAW_INPUT;
    }
};

TEST_F(FpvCamAngleTest, CachedFactorsPersistAcrossCallsWithUnchangedAngle)
{
    rxConfigMutable()->fpvCamAngleDegrees = 45;

    scaleRawSetpointToFpvCamAngle();
    expectRawSetpointMatchesAngle(45);

    // re-apply the same raw inputs; angle is unchanged so the recompute
    // branch is skipped and cached cos/sin factors must still be in effect,
    // not silently reset to the identity (cos=1, sin=0) locals
    rawSetpoint[ROLL] = TEST_ROLL_INPUT;
    rawSetpoint[YAW] = TEST_YAW_INPUT;
    scaleRawSetpointToFpvCamAngle();
    expectRawSetpointMatchesAngle(45);
}

TEST_F(FpvCamAngleTest, FactorsRecomputeWhenAngleChanges)
{
    rxConfigMutable()->fpvCamAngleDegrees = 45;
    scaleRawSetpointToFpvCamAngle();
    expectRawSetpointMatchesAngle(45);

    rawSetpoint[ROLL] = TEST_ROLL_INPUT;
    rawSetpoint[YAW] = TEST_YAW_INPUT;
    rxConfigMutable()->fpvCamAngleDegrees = 90;
    scaleRawSetpointToFpvCamAngle();
    expectRawSetpointMatchesAngle(90);
}

// Feedforward is held between frames; when the frames stop it must not be held until RX loss is detected.
class FeedforwardStallTest : public ::testing::Test {
protected:
    timeDelta_t frameIntervalUs = 4000;

    virtual void SetUp() {
        pgResetAll();                                   // default rates
        rxConfigMutable()->rc_smoothing = 0;            // getFeedforward() gives the raw value
        rxConfigMutable()->midrc = 1500;                // rxConfig has no reset template in this test
        rxConfigMutable()->mincheck = 1050;
        loadControlRateProfile();
        memset(&pidRuntime, 0, sizeof(pidRuntime));
        pidRuntime.feedforwardJitterFactorInv = 1.0f / rcUsSpanToNorm(1.0f + 7);
        pidRuntime.feedforwardBoostFactor = 0.015f;
        initRcProcessing();
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
            rcData[i] = 1500;
        }
        rcData[THROTTLE] = 1000;
        testTimeUs = 1000000;
        for (int i = 0; i < 20; i++) {
            frame();
        }
    }

    void frame(void)
    {
        testTimeUs += frameIntervalUs;
        rxRuntimeState.lastRcFrameTimeUs = testTimeUs;
        updateRcRefreshRate(testTimeUs, true);
        updateRcCommands();
        processRcCommand();
    }

    // a PID loop without a new frame, afterUs after the last frame
    void loopWithoutFrame(timeDelta_t afterUs)
    {
        testTimeUs = rxRuntimeState.lastRcFrameTimeUs + afterUs;
        processRcCommand();
    }
};

TEST_F(FeedforwardStallTest, FastStickMoveGivesFeedforwardHeldUntilTheNextFrame)
{
    rcData[PITCH] = 1580;
    frame();
    const float feedforward = getFeedforward(FD_PITCH);
    EXPECT_GT(fabsf(feedforward), 100.0f);

    loopWithoutFrame(frameIntervalUs - 500);       // between frames: held
    EXPECT_FLOAT_EQ(feedforward, getFeedforward(FD_PITCH));
    loopWithoutFrame(3 * frameIntervalUs);         // a few frames late (jitter): still held
    EXPECT_FLOAT_EQ(feedforward, getFeedforward(FD_PITCH));
}

TEST_F(FeedforwardStallTest, FeedforwardDropsWhenFramesStop)
{
    rcData[PITCH] = 1580;
    frame();
    EXPECT_GT(fabsf(getFeedforward(FD_PITCH)), 100.0f);

    loopWithoutFrame(25000);                       // no frame for 25 ms on a 4 ms link
    EXPECT_FLOAT_EQ(0.0f, getFeedforward(FD_PITCH));
    loopWithoutFrame(100000);
    EXPECT_FLOAT_EQ(0.0f, getFeedforward(FD_PITCH));
}

TEST_F(FeedforwardStallTest, NoStaleFeedforwardWhenFramesComeBack)
{
    rcData[PITCH] = 1580;
    frame();
    loopWithoutFrame(25000);
    ASSERT_FLOAT_EQ(0.0f, getFeedforward(FD_PITCH));

    // the stick did not move while frames were missing
    testTimeUs = rxRuntimeState.lastRcFrameTimeUs + 30000 - frameIntervalUs;
    frame();
    EXPECT_LT(fabsf(getFeedforward(FD_PITCH)), 10.0f);
}

TEST_F(FeedforwardStallTest, SlowLinkIsLeftAlone)
{
    // 12.5 Hz (DJI O3 before arming): the limit is beyond RX loss, feedforward is held as before
    frameIntervalUs = 80000;
    for (int i = 0; i < 5; i++) {
        frame();
    }
    rcData[PITCH] = 1580;
    frame();
    const float feedforward = getFeedforward(FD_PITCH);
    EXPECT_NE(0.0f, feedforward);
    loopWithoutFrame(140000);
    EXPECT_FLOAT_EQ(feedforward, getFeedforward(FD_PITCH));
}
