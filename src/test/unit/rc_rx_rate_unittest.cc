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

    #include "config/config.h"

    #include "drivers/time.h"

    #include "fc/controlrate_profile.h"
    #include "fc/rc.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

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
    static timeUs_t nowUs = 1000000;
    timeUs_t micros(void) { return nowUs; }

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
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Rx rate estimation for rc smoothing and feedforward filters
class RxRateTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        pgResetAll();
        rxConfigMutable()->midrc = 1500;
        rxConfigMutable()->mincheck = 1050;
        loadControlRateProfile();
        memset(&pidRuntime, 0, sizeof(pidRuntime));
        pidRuntime.feedforwardSmoothFactor = (1.0f / 250.0f) * 0.65f / (1.0f - 0.65f);  // as pidInit()
        pidRuntime.feedforwardJitterFactorInv = 1.0f / rcUsSpanToNorm(1.0f + 7);
        initRcProcessing();
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
            rcData[i] = 1500;
        }
        rcData[THROTTLE] = 1000;
        debugMode = DEBUG_FEEDFORWARD_LIMIT;
    }

    void frame(timeDelta_t intervalUs)
    {
        nowUs += intervalUs;
        rxRuntimeState.lastRcFrameTimeUs = nowUs;
        updateRcRefreshRate(nowUs, true);
        updateRcCommands();
        processRcCommand();
    }

    // feedforward pt1 gain x 1000, as last set by updateFeedforwardFilters()
    int feedforwardGain(void) { return debug[6]; }
};

TEST_F(RxRateTest, EvenlyTimedLinkIsMeasured)
{
    for (int i = 0; i < 300; i++) {
        frame(4000);
    }
    EXPECT_NEAR(250.0f, getCurrentRxRateHz(), 250.0f * 0.05f);
}

TEST_F(RxRateTest, AlternatingIntervalsAreMeasured)
{
    // DJI O3 SBUS when armed: frames 4.0 and 6.1 ms apart in turn
    for (int i = 0; i < 300; i++) {
        frame(i % 2 ? 6100 : 4000);
    }
    EXPECT_NEAR(198.0f, getCurrentRxRateHz(), 198.0f * 0.05f);
    const int gainAt198Hz = feedforwardGain();

    // and the filters follow the measured rate: same gain as on an even link at that rate
    SetUp();
    for (int i = 0; i < 300; i++) {
        frame(5050);
    }
    EXPECT_NEAR(feedforwardGain(), gainAt198Hz, 20);
}

TEST_F(RxRateTest, SlowLinkThenAlternatingIntervals)
{
    // before arming the O3 sends a frame every 80 ms, then switches to 4 / 6 ms frames
    for (int i = 0; i < 40; i++) {
        frame(80400);
    }
    for (int i = 0; i < 300; i++) {
        frame(i % 2 ? 6100 : 4000);
    }
    EXPECT_NEAR(198.0f, getCurrentRxRateHz(), 198.0f * 0.05f);
}

TEST_F(RxRateTest, AlternatingIntervalsAfterEvenLink)
{
    // the O3 sends 4 ms frames for a while, so the estimate may start at 250 Hz
    for (int i = 0; i < 300; i++) {
        frame(4000);
    }
    for (int i = 0; i < 300; i++) {
        frame(i % 2 ? 6100 : 4000);
    }
    EXPECT_NEAR(198.0f, getCurrentRxRateHz(), 198.0f * 0.05f);
}

TEST_F(RxRateTest, DroppedFramesKeepRetuning)
{
    // 250 Hz link that drops every fifth frame: 4, 4, 4, 8 ms, from either phase,
    // after an estimate of 250 Hz and of 167 Hz (two pairs with the 8 ms interval)
    const timeDelta_t leadInUs[] = { 4000, 6000 };
    for (const timeDelta_t leadIn : leadInUs) {
        for (int phase = 0; phase < 4; phase++) {
            for (int i = 0; i < 100; i++) {
                frame(leadIn);
            }
            debugMode = DEBUG_RC_SMOOTHING_RATE;
            int updates = 0;
            for (int i = 0; i < 400; i++) {
                frame((i + phase) % 4 == 3 ? 8000 : 4000);
                if (i >= 200) {
                    updates += debug[3];  // cutoffs updated on this frame
                }
            }
            EXPECT_NEAR(250.0f, getCurrentRxRateHz(), 250.0f * 0.05f);
            EXPECT_GT(updates, 20);
        }
    }
}
