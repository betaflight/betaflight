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

    #include "rx/rx.h"

    #include "sensors/battery.h"

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
