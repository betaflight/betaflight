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
#include <string.h>
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

    #include "fc/controlrate_profile.h"
    #include "fc/rc.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "config/config.h"

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
    pidRuntime_t pidRuntime;
    uint32_t targetPidLooptime;
    gyro_t gyro;

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

    extern float rawSetpoint[XYZ_AXIS_COUNT];
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const uint32_t PID_LOOPTIME_US = 250;  // 4kHz

static const float TEST_THROTTLE_PWM = 1500.0f;
static const float TEST_ROLL_PWM = 1700.0f;

// rc.c keeps the last frame time in a function-local static, so time has to
// keep moving forward across tests in this binary
static timeUs_t nowUs = 1000000;

// Feed frames spaced frameIntervalUs apart and run the PID-rate part of
// processRcCommand() between them, as the scheduler does on the FC
static void runFrames(int frameCount, timeDelta_t frameIntervalUs)
{
    for (int frame = 0; frame < frameCount; frame++) {
        nowUs += frameIntervalUs;
        rxRuntimeState.lastRcFrameTimeUs = nowUs;
        rcData[ROLL] = TEST_ROLL_PWM;
        rcData[PITCH] = 1500.0f;
        rcData[YAW] = 1500.0f;
        rcData[THROTTLE] = TEST_THROTTLE_PWM;

        updateRcRefreshRate(nowUs, true);
        updateRcCommands();
        processRcCommand();

        for (timeDelta_t t = PID_LOOPTIME_US; t < frameIntervalUs; t += PID_LOOPTIME_US) {
            processRcCommand();
        }
    }
}

class RcSmoothingTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        PG_RESET(rxConfig);
        rxConfigMutable()->rc_smoothing = 1;
        rxConfigMutable()->rc_smoothing_setpoint_cutoff = 0;  // auto
        rxConfigMutable()->rc_smoothing_throttle_cutoff = 0;  // auto
        rxConfigMutable()->rc_smoothing_auto_factor_rpy = 30;
        rxConfigMutable()->rc_smoothing_auto_factor_throttle = 30;
        rxConfigMutable()->midrc = 1500;
        rxConfigMutable()->mincheck = 1050;

        PG_RESET(rcControlsConfig);
        PG_RESET(systemConfig);
        PG_RESET(controlRateProfiles);
        loadControlRateProfile();

        bootRcProcessing();
    }

public:
    static void bootRcProcessing(void)
    {
        // the smoothing filters live in zero-initialised memory on the FC;
        // start every test from that power-on state
        memset(getRcSmoothingData(), 0, sizeof(rcSmoothingFilter_t));
        // at boot readEEPROM() runs initRcProcessing() before pidInit()
        // has set the pid looptime
        targetPidLooptime = 0;
        initRcProcessing();
        targetPidLooptime = PID_LOOPTIME_US;
    }
};

// SBUS from a DJI O3 without fast mode: one frame every 80ms (12.5Hz),
// which is longer than the 65.5ms that is accepted as a valid rx interval
TEST_F(RcSmoothingTest, SlowRxDoesNotFreezeThrottleAndSetpoint)
{
    runFrames(20, 80000);

    // rcCommand[THROTTLE] is normalised to 0..1, rcData 1500 maps to ~0.47
    EXPECT_GT(rcCommand[THROTTLE], 0.4f);
    EXPECT_GT(getSetpointRate(ROLL), 0.5f * rawSetpoint[ROLL]);
    EXPECT_NEAR(rawSetpoint[ROLL], getSetpointRate(ROLL), 1.0f);
}

TEST_F(RcSmoothingTest, SlowRxWithManualCutoffsDoesNotFreeze)
{
    rxConfigMutable()->rc_smoothing_setpoint_cutoff = 20;
    rxConfigMutable()->rc_smoothing_throttle_cutoff = 20;
    bootRcProcessing();

    runFrames(20, 80000);

    EXPECT_GT(rcCommand[THROTTLE], 0.4f);
    EXPECT_NEAR(rawSetpoint[ROLL], getSetpointRate(ROLL), 1.0f);
    EXPECT_EQ(20, getRcSmoothingData()->setpointCutoffFrequency);
    EXPECT_EQ(20, getRcSmoothingData()->throttleCutoffFrequency);
}

TEST_F(RcSmoothingTest, SmoothingWorksBeforeFirstRateMeasurement)
{
    // the very first frames, before three valid intervals were seen,
    // must already be passed through the filters (the cutoff depends on
    // the rate left behind by earlier tests, so only check for movement)
    runFrames(2, 4000);

    EXPECT_GT(rcCommand[THROTTLE], 0.0f);
    EXPECT_GT(getSetpointRate(ROLL), 0.0f);
}

// auto cutoffs for a slow link follow the measured rate (constrained to the
// 15.26Hz lower limit of the rate range), not the 100Hz start-up guess
TEST_F(RcSmoothingTest, SlowRxAutoCutoffUsesConstrainedRate)
{
    runFrames(20, 80000);

    EXPECT_NEAR(1e6f / 65500, getCurrentRxRateHz(), 0.1f);
    EXPECT_EQ(15, getRcSmoothingData()->setpointCutoffFrequency);
    EXPECT_EQ(15, getRcSmoothingData()->throttleCutoffFrequency);
}

// regression guard for normal link rates: auto cutoff = rate * 1.5 / (1 + 30 / 10)
TEST_F(RcSmoothingTest, NormalRxAutoCutoffFollowsRate)
{
    runFrames(100, 4000);  // 250Hz

    EXPECT_NEAR(250.0f, getCurrentRxRateHz(), 1.0f);
    EXPECT_EQ(93, getRcSmoothingData()->setpointCutoffFrequency);
    EXPECT_EQ(93, getRcSmoothingData()->throttleCutoffFrequency);
    EXPECT_GT(rcCommand[THROTTLE], 0.4f);
    EXPECT_NEAR(rawSetpoint[ROLL], getSetpointRate(ROLL), 1.0f);

    runFrames(100, 20000);  // 50Hz

    EXPECT_NEAR(50.0f, getCurrentRxRateHz(), 1.0f);
    EXPECT_EQ(18, getRcSmoothingData()->setpointCutoffFrequency);  // 18.75, MAX(15, ...)
}
