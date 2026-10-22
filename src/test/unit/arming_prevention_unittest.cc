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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

extern "C" {

    #include "blackbox/blackbox.h"

    #include "build/debug.h"

    #include "common/filter.h"
    #include "common/maths.h"

    #include "config/config.h"
    #include "config/feature.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/failsafe.h"
    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"
    #include "flight/servos.h"

    #include "io/beeper.h"
    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/gps_rescue.h"
    #include "pg/motor.h"
    #include "pg/rx.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "rx/rx.h"

    #include "scheduler/scheduler.h"

    #include "sensors/acceleration.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"

    #include "telemetry/telemetry.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
    PG_REGISTER(compassConfig_t, compassConfig, PG_COMPASS_CONFIG, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(pidConfig_t, pidConfig, PG_PID_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);
    PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
    PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);

    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint16_t averageSystemLoadPercent = 0;
    uint8_t cliMode = 0;
    uint8_t debugMode = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    static pidProfile_t defaultTestPidProfile;   // zeroed: launch_control_mode NORMAL
    pidProfile_t *currentPidProfile = &defaultTestPidProfile;
    controlRateConfig_t *currentControlRateProfile;
    attitudeEulerAngles_t attitude;

    gpsSolutionData_t gpsSol;
    uint16_t GPS_distanceToHome;
    gpsLocation_t GPS_home_llh;
    uint32_t GPS_distanceToHomeCm = 0;
    int16_t GPS_directionToHome = 0;

    uint32_t targetPidLooptime;
    bool cmsInMenu = false;
    float axisPID_P[3], axisPID_I[3], axisPID_D[3], axisPIDSum[3];
    rxRuntimeState_t rxRuntimeState = {};
    acc_t acc = {};
    gyro_t gyro = {};
    matrix33_t rMat = {};
    float motor[MAX_SUPPORTED_MOTORS];
    bool useDshotTelemetry = false;
    bool mockRpmTelemetry = false;
    float mockRpm[MAX_SUPPORTED_MOTORS];
    bool mockIsUpright = false;
    float mockCosTiltAngle = 1.0f;   // level
    pidAxisData_t pidData[3];
    uint16_t mockGyroAbsRateDps = 0;
    uint8_t activePidLoopDenom = 1;

    float getGpsDataIntervalSeconds(void) { return 0.1f; }
    float getGpsDataFrequencyHz(void) { return 10.0f; }
}

uint32_t simulationFeatureFlags = 0;
uint32_t simulationTime = 0;
bool gyroCalibDone = false;
bool simulationHaveRx = false;
bool simulationFailsafeActive = false;
pidStabilisationState_e simulationPidStabilisationState = PID_STABILISATION_OFF;

#include "gtest/gtest.h"

TEST(ArmingPreventionTest, CalibrationPowerOnGraceAngleThrottleArmSwitch)
{
    // given
    simulationTime = 0;
    gyroCalibDone = false;
    sensorsSet(SENSOR_GYRO);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    // default channel positions
    rcData[THROTTLE] = 1400;
    rcData[4] = 1800;

    // and
    systemConfigMutable()->powerOnArmingGraceTime = 5;
    setArmingDisabled(ARMING_DISABLED_BOOT_GRACE_TIME);

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_ANGLE | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // gyro calibration is done
    gyroCalibDone = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ANGLE | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // quad is level
    mockIsUpright = true;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    rcData[THROTTLE] = 1000;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arming grace time has elapsed
    simulationTime += systemConfig()->powerOnArmingGraceTime * 1e6;

    // when
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    rcData[4] = 1000;

    // when
    // arm guard time elapses
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, ArmingGuardRadioLeftOnAndArmed)
{
    // given
    simulationTime = 0;
    gyroCalibDone = false;
    sensorsSet(SENSOR_GYRO);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_CALIBRATING, getArmingDisableFlags());

    // given
    // arm channel takes a safe default value from the RX after power on
    rcData[4] = 1500;

    // and
    // a short time passes while calibration is in progress
    simulationTime += 1e6;

    // and
    // during calibration RF link is established and ARM switch is on
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_CALIBRATING | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // calibration is done
    gyroCalibDone = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch is switched off by user
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // arming enabled as arm switch has been off for sufficient time
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, Prearm)
{
    // given
    simulationTime = 0;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPREARM;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_NOPREARM, getArmingDisableFlags());

    // given
    // prearm is enabled
    rcData[5] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // arming enabled as arm switch has been off for sufficient time
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
}

TEST(ArmingPreventionTest, RadioTurnedOnAtAnyTimeArmed)
{
    // given
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;

    // and
    // RX has no link to radio
    simulationHaveRx = false;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // RF link is established and arm switch is turned on on radio
    simulationHaveRx = true;
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_NOT_DISARMED | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off by user
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, In3DModeAllowArmingWhenEnteringThrottleDeadband)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->midrc = 1500;
    flight3DConfigMutable()->deadband3d_throttle = 5;

    // and
    rcData[THROTTLE] = 1400;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // throttle moved to centre
    rcData[THROTTLE] = 1496;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, When3DModeDisabledThenNormalThrottleArmingConditionApplies)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOX3D;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;
    rxConfigMutable()->midrc = 1500;
    flight3DConfigMutable()->deadband3d_throttle = 5;

    // and
    // safe throttle value for 3D mode
    rcData[THROTTLE] = 1500;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // and
    // disable 3D mode is off (i.e. 3D mode is on)
    rcData[5] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // disable 3D mode
    rcData[5] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // throttle moved low
    rcData[THROTTLE] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, WhenUsingSwitched3DModeThenNormalThrottleArmingConditionApplies)
{
    // given
    simulationFeatureFlags = FEATURE_3D; // Using 3D mode
    simulationTime = 30e6; // 30 seconds after boot
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOX3D;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // and
    rcData[THROTTLE] = 1000;
    mockIsUpright = true;
    simulationHaveRx = true;

    // and
    // arm channel has a safe default value
    rcData[4] = 1100;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // raise throttle to unsafe position
    rcData[THROTTLE] = 1500;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE, getArmingDisableFlags());

    // given
    // attempt to arm
    rcData[4] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_THROTTLE | ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // throttle moved low
    rcData[THROTTLE] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH, getArmingDisableFlags());

    // given
    // arm switch turned off
    rcData[4] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
}

TEST(ArmingPreventionTest, GPSRescueWithoutFixPreventsArm)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXGPSRESCUE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH|ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_GPS, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // receive GPS fix
    ENABLE_STATE(GPS_FIX);

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));
}

TEST(ArmingPreventionTest, GPSRescueSwitchPreventsArm)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;
    gpsSol.numSat = 5;
    ENABLE_STATE(GPS_FIX);

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXGPSRESCUE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Start out with rescue enabled
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_ARM_SWITCH|ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_RESC, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disable Rescue
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXGPSRESCUE));
}

TEST(ArmingPreventionTest, ParalyzeOnAtBoot)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPARALYZE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Paralyze on at boot
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // when
    updateActivatedModes();

    // expect
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
}

TEST(ArmingPreventionTest, Paralyze)
{
    // given
    simulationFeatureFlags = 0;
    simulationTime = 0;
    gyroCalibDone = true;

    // and
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXPARALYZE;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXBEEPERON;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(3)->modeId = BOXVTXPITMODE;
    modeActivationConditionsMutable(3)->linkedTo = BOXPARALYZE;
    rcControlsInit();

    // and
    rxConfigMutable()->mincheck = 1050;

    // given
    rcData[THROTTLE] = 1000;
    rcData[AUX1] = 1000;
    rcData[AUX2] = 1800; // Start out with paralyze enabled
    rcData[AUX3] = 1000;
    mockIsUpright = true;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // arm
    rcData[AUX1] = 1800;

    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());

    // given
    // disarm
    rcData[AUX1] = 1000;

    // when
    disarm(DISARM_REASON_SYSTEM);
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // given
    simulationTime = 10e6; // 10 seconds after boot

    // when
    updateActivatedModes();

    // expect
    EXPECT_FALSE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0, getArmingDisableFlags());
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXPARALYZE));

    // given
    // disable paralyze once after the startup timer
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();

    // enable paralyze again
    rcData[AUX2] = 1800;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_PARALYZE, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_FALSE(IS_RC_MODE_ACTIVE(BOXBEEPERON));

    // given
    // enable beeper
    rcData[AUX3] = 1800;

    // when
    updateActivatedModes();

    // expect
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXBEEPERON));

    // given
    // try exiting paralyze mode and ensure arming and pit mode are still disabled
    rcData[AUX2] = 1000;

    // when
    updateActivatedModes();
    updateArmingStatus();

    // expect
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(ARMING_DISABLED_PARALYZE, getArmingDisableFlags());
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXPARALYZE));
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
}

// ---------------------------------------------------------------------------
// Launch Control, LIFT mode, on one 3-position switch (SC):
//   down = regular flight, mid = staged (LAUNCH CONTROL), high = send it (+ LAUNCH TRIGGER)
// ---------------------------------------------------------------------------

#define LIFT_TIME_MS 1000
#define STICK_LOW     1000
#define STICK_TAKEOFF 1500   // comfortably above the 20% trigger
#define ARM_OFF 1000
#define ARM_ON  1800
#define SC_DOWN 1000
#define SC_MID  1500
#define SC_HIGH 2000

static pidProfile_t liftTestPidProfile;

static void advanceMs(uint32_t ms)
{
    simulationTime += ms * 1000;
    updateActivatedModes();
    processRx(simulationTime);
    launchControlLiftUpdate(simulationTime);
}

static void setSC(uint16_t position)
{
    rcData[AUX2] = position;
    advanceMs(20);   // one RX frame or so
}

static void arm(void)
{
    rcData[AUX1] = ARM_ON;
    updateActivatedModes();
    tryArm();
    advanceMs(20);
}

static void disarmNow(void)
{
    disarm(DISARM_REASON_SWITCH);
    rcData[AUX1] = ARM_OFF;
    advanceMs(20);
}

// the attitude the IMU reports: level, or rolled upside down
static void setAttitude(bool upsideDown)
{
    memset(&rMat, 0, sizeof(rMat));
    rMat.m[0][0] = 1.0f;
    rMat.m[1][1] = upsideDown ? -1.0f : 1.0f;
    rMat.m[2][2] = upsideDown ? -1.0f : 1.0f;
}

// specific force along earth-up [g]: 1 sitting still, more while thrust speeds the climb,
// below zero while braking upside down
static void setVerticalG(float g)
{
    acc.dev.acc_1G = 2048;
    acc.dev.acc_1G_rec = 1.0f / 2048;
    acc.accADC.x = 0.0f;
    acc.accADC.y = 0.0f;
    acc.accADC.z = g * 2048 * rMat.m[2][2];   // earth-up is body z, or body -z upside down
}

static void setUpLift(bool withTriggerMode = true)
{
    // clean slate: nothing left over from the other tests
    DISABLE_ARMING_FLAG(ARMED);
    unsetArmingDisabled((armingDisableFlags_e)0xFFFFFFFF);
    flightModeFlags = 0;
    simulationFeatureFlags = 0;
    simulationFailsafeActive = false;
    simulationHaveRx = true;
    gyroCalibDone = true;
    mockIsUpright = true;
    mockCosTiltAngle = 1.0f;
    acc.jerkMagnitude = 0.0f;
    setAttitude(false);
    setVerticalG(1.0f);   // sitting still
    memset(motor, 0, sizeof(motor));
    memset(mockRpm, 0, sizeof(mockRpm));
    useDshotTelemetry = false;
    mockRpmTelemetry = false;
    mockGyroAbsRateDps = 0;
    memset(pidData, 0, sizeof(pidData));
    pidConfigMutable()->runaway_takeoff_prevention = 0;
    rxConfigMutable()->airModeActivateThreshold = 25;
    sensorsSet(SENSOR_ACC);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyro.gyroADCf[axis] = 0.0f;
    }
    simulationTime = 10 * 1000000;

    memset(modeActivationConditionsMutable(0), 0, sizeof(modeActivationCondition_t) * MAX_MODE_ACTIVATION_CONDITION_COUNT);
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;      // SC mid + high
    modeActivationConditionsMutable(1)->modeId = BOXLAUNCHCONTROL;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    if (withTriggerMode) {
        modeActivationConditionsMutable(2)->auxChannelIndex = 1;  // SC high
        modeActivationConditionsMutable(2)->modeId = BOXLAUNCHTRIGGER;
        modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
        modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);
    }
    rcControlsInit();
    rxConfigMutable()->mincheck = 1050;

    memset(&liftTestPidProfile, 0, sizeof(liftTestPidProfile));
    liftTestPidProfile.launchControlMode = LAUNCH_CONTROL_MODE_LIFT;
    liftTestPidProfile.launchControlThrottlePercent = 20;
    liftTestPidProfile.launchControlAllowTriggerReset = true;
    liftTestPidProfile.launchControlLiftTime = LIFT_TIME_MS;
    liftTestPidProfile.launchControlLiftThrottle = 100;
    liftTestPidProfile.launchControlLiftFlipDirection = LAUNCH_LIFT_FLIP_AUTO;
    liftTestPidProfile.launchControlLiftFlipRate = LAUNCH_CONTROL_LIFT_FLIP_RATE_DEFAULT_DPS;
    liftTestPidProfile.launchControlLiftBrakeTime = LAUNCH_CONTROL_LIFT_BRAKE_TIME_DEFAULT_MS;
    currentPidProfile = &liftTestPidProfile;

    // disarmed with SC down resets any earlier launch
    rcData[THROTTLE] = STICK_LOW;
    rcData[ROLL] = rcData[PITCH] = rcData[YAW] = 1500;
    rcData[AUX1] = ARM_OFF;
    rcData[AUX2] = SC_DOWN;
    updateActivatedModes();
    updateArmingStatus();
    processRx(simulationTime);
}

// armed with SC in mid: staged in LNCH, ready to send
static void armAndStage(void)
{
    setUpLift();
    setSC(SC_MID);
    arm();
}

TEST(LaunchControlLiftTest, DownAtArmingIsRegularFlight)
{
    setUpLift();
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());
    EXPECT_FALSE(isLaunchControlPreStaged());
    arm();

    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_FALSE(isLaunchControlActive());   // no LNCH

    // as with every launch control mode, the switch only counts at arming:
    // mid or high in the air does nothing, and never pins the motors at idle
    setSC(SC_MID);
    EXPECT_FALSE(isLaunchControlActive());
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());   // nothing covering the warnings in flight
}

TEST(LaunchControlLiftTest, PreArmChecklist)
{
    setUpLift();

    setSC(SC_MID);
    EXPECT_STREQ("LIFT PRE-STAGE", getLaunchControlLiftPreArmMessage());
    EXPECT_TRUE(isLaunchControlPreStaged());   // mode shows LNCH before arming

    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    advanceMs(20);
    EXPECT_STREQ("LIFT: ACRO ONLY", getLaunchControlLiftPreArmMessage());
    EXPECT_FALSE(isLaunchControlPreStaged());
    DISABLE_FLIGHT_MODE(ANGLE_MODE);

    setSC(SC_HIGH);
    EXPECT_STREQ("LIFT: TRIGGER OFF FIRST", getLaunchControlLiftPreArmMessage());

    setSC(SC_DOWN);
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());
    EXPECT_FALSE(isLaunchControlPreStaged());

    setUpLift(false);   // no LAUNCH TRIGGER mode set up
    setSC(SC_MID);
    EXPECT_STREQ("LIFT: NO TRIGGER MODE", getLaunchControlLiftPreArmMessage());
    EXPECT_FALSE(isLaunchControlPreStaged());
}

TEST(LaunchControlLiftTest, ArmingInMidStages)
{
    armAndStage();
    EXPECT_TRUE(isLaunchControlActive());    // LNCH, "LIFT STAGED"
    EXPECT_FALSE(isLaunchControlLiftAwaitingTriggerOff());
    EXPECT_FALSE(isLaunchControlPreStaged());
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());

    rcData[THROTTLE] = 1100;   // below the 20% takeoff threshold: keeps holding
    advanceMs(2000);
    EXPECT_TRUE(isLaunchControlActive());
    EXPECT_FALSE(isLaunchControlLifting());
    rcData[THROTTLE] = STICK_LOW;
}

TEST(LaunchControlLiftTest, HighSendsItForTheSetTimeThenHandsOverToTheSticks)
{
    armAndStage();

    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlActive());
    EXPECT_TRUE(isLaunchControlLifting());
    EXPECT_FLOAT_EQ(1.0f, getLaunchControlLiftThrottle());

    // stick at the bottom the whole time: the lift carries on regardless,
    // and the PIDs stay on even though the stick throttle is low
    advanceMs(500);   // the lift clock starts on the RX frame that sees the trigger
    EXPECT_TRUE(isLaunchControlLifting());
    EXPECT_EQ(PID_STABILISATION_ON, simulationPidStabilisationState);
    EXPECT_EQ(500u, getLaunchControlLiftRemainingMs());

    advanceMs(LIFT_TIME_MS - 500 - 1);
    EXPECT_TRUE(isLaunchControlLifting());

    // time's up: the sticks have it (mode back to AIR), and the OSD says so for a moment
    advanceMs(1);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlActive());
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
    advanceMs(LAUNCH_CONTROL_LIFT_HANDOVER_NOTICE_MS);
    EXPECT_FALSE(isLaunchControlLiftHandoverRecent());

    // one launch per arm: working SC in flight does nothing
    setSC(SC_MID);
    setSC(SC_HIGH);
    setSC(SC_DOWN);
    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlActive());
}

TEST(LaunchControlLiftTest, HandoverNoticeDoesNotComeBackAfterTheTimerWraps)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(LIFT_TIME_MS);
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
    advanceMs(LAUNCH_CONTROL_LIFT_HANDOVER_NOTICE_MS);
    EXPECT_FALSE(isLaunchControlLiftHandoverRecent());

    // still flying half an hour later: the signed microsecond delta has gone negative,
    // and the notice must not return and cover the RSSI / LAND NOW warnings
    for (int minutes = 1; minutes <= 71; minutes++) {
        advanceMs(60 * 1000);
        ASSERT_FALSE(isLaunchControlLiftHandoverRecent()) << "returned after " << minutes << " minutes";
    }
    // about 71.6 minutes on, the 32-bit microsecond timer has gone all the way round and the
    // old handover time reads as recent again, unless it was forgotten when the notice ended
    for (int step = 0; step < 400; step++) {
        advanceMs(100);
        ASSERT_FALSE(isLaunchControlLiftHandoverRecent()) << "returned " << step * 100 << " ms after 71 minutes";
    }
}

TEST(LaunchControlLiftTest, UsesTheConfiguredLiftThrottleAndTime)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftThrottle = 60;
    liftTestPidProfile.launchControlLiftTime = 2500;

    setSC(SC_HIGH);
    EXPECT_FLOAT_EQ(0.6f, getLaunchControlLiftThrottle());
    advanceMs(2500 - 1);
    EXPECT_TRUE(isLaunchControlLifting());
    advanceMs(1);
    EXPECT_FALSE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, SticksDoNotEndTheLift)
{
    armAndStage();
    setSC(SC_HIGH);

    rcData[ROLL] = 2000;
    rcData[PITCH] = 1000;
    rcData[YAW] = 2000;
    rcData[THROTTLE] = 2000;
    advanceMs(300);
    EXPECT_TRUE(isLaunchControlLifting());
    rcData[THROTTLE] = STICK_LOW;
    advanceMs(300);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, BackToMidAbortsToTheSticks)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(200);

    setSC(SC_MID);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlActive());   // not back to LNCH: same as the end of a run
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
}

TEST(LaunchControlLiftTest, AllTheWayDownAbortsToTheSticks)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(200);

    setSC(SC_DOWN);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
}

TEST(LaunchControlLiftTest, ArmingInHighNeverLaunches)
{
    setUpLift();
    rcData[AUX2] = SC_HIGH;
    arm();

    EXPECT_TRUE(isLaunchControlActive());
    EXPECT_TRUE(isLaunchControlLiftAwaitingTriggerOff());
    advanceMs(2000);
    EXPECT_FALSE(isLaunchControlLifting());

    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, DownBeforeLaunchStandsDownForThisArm)
{
    armAndStage();

    setSC(SC_DOWN);
    EXPECT_FALSE(isLaunchControlActive());   // regular flight: no LNCH hold

    setSC(SC_MID);                            // staging only happens at arming
    EXPECT_FALSE(isLaunchControlActive());
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());

    // disarm, re-arm in mid: staged again
    disarmNow();
    setSC(SC_MID);
    arm();
    EXPECT_TRUE(isLaunchControlActive());
}

TEST(LaunchControlLiftTest, TakingOffFromLnchOnTheThrottleSkipsTheLift)
{
    armAndStage();

    rcData[THROTTLE] = STICK_TAKEOFF;
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlActive());

    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    rcData[THROTTLE] = STICK_LOW;
}

TEST(LaunchControlLiftTest, OnlyFromAcro)
{
    // armed in Angle with SC mid: not staged (the checklist warned before arming)
    setUpLift();
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    setSC(SC_MID);
    arm();
    EXPECT_FALSE(isLaunchControlActive());
    disarmNow();
    DISABLE_FLIGHT_MODE(ANGLE_MODE);

    // staged in acro, then Angle (or Chirp) switched on: the trigger is refused, not saved
    arm();
    EXPECT_TRUE(isLaunchControlActive());
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    DISABLE_FLIGHT_MODE(ANGLE_MODE);
    advanceMs(500);
    EXPECT_FALSE(isLaunchControlLifting());

    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, NoTriggerModeMeansNoStaging)
{
    setUpLift(false);
    setSC(SC_MID);
    arm();
    EXPECT_FALSE(isLaunchControlActive());
}

TEST(LaunchControlLiftTest, FailsafeEndsTheLift)
{
    armAndStage();
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());

    simulationFailsafeActive = true;
    advanceMs(10);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftHandoverRecent());   // failsafe is flying, not the sticks
    simulationFailsafeActive = false;
}

TEST(LaunchControlLiftTest, TimerRunsInThePidLoopEvenWithoutRxFrames)
{
    armAndStage();
    setSC(SC_HIGH);

    // no processRx calls at all from here: only the PID-loop update runs
    simulationTime += LIFT_TIME_MS * 1000;
    launchControlLiftUpdate(simulationTime);
    EXPECT_FALSE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, AfterALaunchScDownWhileDisarmedResetsIt)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(LIFT_TIME_MS);
    disarmNow();

    // disarmed with SC still up: the checklist says why it won't stage
    EXPECT_STREQ("LIFT USED PEAK 0: SWITCH OFF", getLaunchControlLiftPreArmMessage());
    EXPECT_FALSE(isLaunchControlPreStaged());
    arm();
    EXPECT_FALSE(isLaunchControlActive());
    disarmNow();

    // SC down while disarmed resets it (launch_trigger_allow_reset = ON)
    setSC(SC_DOWN);
    setSC(SC_MID);
    EXPECT_STREQ("LIFT PRE-STAGE", getLaunchControlLiftPreArmMessage());
    arm();
    EXPECT_TRUE(isLaunchControlActive());
}

TEST(LaunchControlLiftTest, DisarmMidLiftCountsAsTheLaunch)
{
    armAndStage();
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());

    disarmNow();
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_STREQ("LIFT USED PEAK 0: SWITCH OFF", getLaunchControlLiftPreArmMessage());
}

TEST(LaunchControlLiftTest, OsdGettersDoNotTouchThePidProfile)
{
    // The OSD element and warning tasks call these. They must be plain reads of state
    // cached by the flight controller tasks: no profile dereference, no arming logic.
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(100);

    pidProfile_t *saved = currentPidProfile;
    currentPidProfile = NULL;

    EXPECT_TRUE(isLaunchControlLifting());
    EXPECT_FLOAT_EQ(1.0f, getLaunchControlLiftThrottle());
    EXPECT_GT(getLaunchControlLiftRemainingMs(), 0u);
    EXPECT_FALSE(isLaunchControlPreStaged());
    EXPECT_FALSE(isLaunchControlLiftStaged());
    EXPECT_FALSE(isLaunchControlLiftAwaitingTriggerOff());
    EXPECT_FALSE(isLaunchControlLiftHandoverRecent());
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());

    currentPidProfile = saved;
}

TEST(LaunchControlLiftTest, OtherLaunchModesAreUnchanged)
{
    setUpLift();
    liftTestPidProfile.launchControlMode = LAUNCH_CONTROL_MODE_NORMAL;

    // no LIFT checklist for the other modes, but LNCH still shows pre-arm
    setSC(SC_MID);
    EXPECT_EQ(NULL, getLaunchControlLiftPreArmMessage());
    EXPECT_TRUE(isLaunchControlPreStaged());

    // switch on at arming: holds, the trigger mode does nothing, the throttle launches with no lift
    arm();
    EXPECT_TRUE(isLaunchControlActive());
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlActive());
    rcData[THROTTLE] = STICK_TAKEOFF;
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlActive());
    EXPECT_FALSE(isLaunchControlLifting());
    rcData[THROTTLE] = STICK_LOW;
}

static float cosDeg(float deg)
{
    return cosf(deg * (float)M_PI / 180.0f);
}

TEST(LaunchControlLiftTest, NotLevelRefusesTheTriggerAndUsesThePress)
{
    armAndStage();

    // on its side, upside down, or on a steep stand: shown, and the press does nothing
    mockCosTiltAngle = cosDeg(LAUNCH_CONTROL_LIFT_MAX_START_TILT_DEG + 5);
    advanceMs(20);
    EXPECT_TRUE(isLaunchControlLiftNotLevel());     // "LIFT: NOT LEVEL"
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlActive());           // still staged, holding on the ground

    // levelled with the trigger still up: must not launch by surprise
    mockCosTiltAngle = 1.0f;
    advanceMs(200);
    EXPECT_FALSE(isLaunchControlLiftNotLevel());
    EXPECT_FALSE(isLaunchControlLifting());

    // a fresh press sends it
    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, MildSlopeIsAllowed)
{
    armAndStage();
    mockCosTiltAngle = cosDeg(LAUNCH_CONTROL_LIFT_MAX_START_TILT_DEG - 5);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());   // self-level sorts it out in the climb
}

TEST(LaunchControlLiftTest, NoAttitudeNoLift)
{
    setUpLift();
    mockIsUpright = false;          // attitude not established (or past small_angle)
    setSC(SC_MID);
    EXPECT_STREQ("LIFT: NOT LEVEL", getLaunchControlLiftPreArmMessage());

    setUpLift();
    sensorsClear(SENSOR_ACC);
    setSC(SC_MID);
    EXPECT_STREQ("LIFT: NEEDS ACC", getLaunchControlLiftPreArmMessage());
    sensorsSet(SENSOR_ACC);
}

extern "C" {
    extern bool mockAccCalibrated;
}

TEST(LaunchControlLiftTest, AnUncalibratedAccelerometerBlocksArming)
{
    // LIFT levels, checks the tilt and measures the climb with the accelerometer,
    // so like Angle mode it needs it calibrated
    setUpLift();
    mockAccCalibrated = false;
    updateArmingStatus();
    EXPECT_TRUE(getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION);

    // the other launch control modes only need it for an angle limit
    liftTestPidProfile.launchControlMode = LAUNCH_CONTROL_MODE_NORMAL;
    updateArmingStatus();
    EXPECT_FALSE(getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION);

    mockAccCalibrated = true;
    liftTestPidProfile.launchControlMode = LAUNCH_CONTROL_MODE_LIFT;
    updateArmingStatus();
    EXPECT_FALSE(getArmingDisableFlags() & ARMING_DISABLED_ACC_CALIBRATION);
}

TEST(LaunchControlLiftTest, BriefUpsetIsRiddenOut)
{
    armAndStage();
    setSC(SC_HIGH);

    // a gust, or leaving uneven ground: past the limits, but not for long
    mockCosTiltAngle = cosDeg(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10);
    gyro.gyroADCf[FD_ROLL] = LAUNCH_CONTROL_LIFT_ABORT_RATE_DPS + 100;
    advanceMs(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS - 50);
    EXPECT_TRUE(isLaunchControlLifting());

    mockCosTiltAngle = 1.0f;
    gyro.gyroADCf[FD_ROLL] = 0.0f;
    advanceMs(20);
    // and again: the clock restarted when it recovered
    mockCosTiltAngle = cosDeg(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10);
    advanceMs(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS - 50);
    EXPECT_TRUE(isLaunchControlLifting());
    mockCosTiltAngle = 1.0f;

    advanceMs(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftAborted());   // a normal handover: "STICKS LIVE"
}

TEST(LaunchControlLiftTest, CantHoldLevelAbortsToTheSticks)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(100);

    // loose prop: it tips over and keeps going
    mockCosTiltAngle = cosDeg(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10);
    for (int ms = 10; ms < LAUNCH_CONTROL_LIFT_ABORT_TIME_MS; ms += 10) {
        advanceMs(10);
        ASSERT_TRUE(isLaunchControlLifting()) << "aborted after only " << ms << " ms";
    }
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlLifting());        // sticks have it, well before the timer
    EXPECT_TRUE(ARMING_FLAG(ARMED));               // not disarmed: the pilot decides
    EXPECT_TRUE(isLaunchControlLiftAborted());     // "LIFT ABORT: STICKS LIVE"
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());

    // used up, as for any other ending
    mockCosTiltAngle = 1.0f;
    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
}

TEST(LaunchControlLiftTest, SpinningAbortsToTheSticks)
{
    armAndStage();
    setSC(SC_HIGH);

    // level, but a prop has let go and it's yawing away
    gyro.gyroADCf[FD_YAW] = -(LAUNCH_CONTROL_LIFT_ABORT_RATE_DPS + 200);
    for (int ms = 0; ms <= LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 20; ms += 10) {
        advanceMs(10);
    }
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftAborted());
    gyro.gyroADCf[FD_YAW] = 0.0f;
}

TEST(LaunchControlLiftTest, AbortFlagClearsOnTheNextLift)
{
    armAndStage();
    setSC(SC_HIGH);
    gyro.gyroADCf[FD_PITCH] = LAUNCH_CONTROL_LIFT_ABORT_RATE_DPS + 1;
    for (int ms = 0; ms <= LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 20; ms += 10) {
        advanceMs(10);
    }
    EXPECT_TRUE(isLaunchControlLiftAborted());
    gyro.gyroADCf[FD_PITCH] = 0.0f;

    // land, disarm, switch off to reset, and go again
    disarmNow();
    setSC(SC_DOWN);
    setSC(SC_MID);
    arm();
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftTest, ImpactIsOffByDefault)
{
    armAndStage();
    setSC(SC_HIGH);
    acc.jerkMagnitude = 5000.0f;     // an enormous jolt: still just logged, not acted on
    advanceMs(20);
    EXPECT_TRUE(isLaunchControlLifting());
    acc.jerkMagnitude = 0.0f;
}

TEST(LaunchControlLiftTest, ImpactAboveTheSettingHandsTheSticksBackAtOnce)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftImpact = 50;   // 500 g/s
    setSC(SC_HIGH);

    // thrust alone: hard, but under the setting
    acc.jerkMagnitude = 480.0f;
    advanceMs(100);
    EXPECT_TRUE(isLaunchControlLifting());

    // the ceiling: no 150 ms wait, and never a disarm
    acc.jerkMagnitude = 900.0f;
    advanceMs(1);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(ARMING_FLAG(ARMED));
    EXPECT_TRUE(isLaunchControlLiftImpacted());   // "LIFT IMPACT: STICKS LIVE"
    EXPECT_TRUE(isLaunchControlLiftAborted());
    acc.jerkMagnitude = 0.0f;
}

TEST(LaunchControlLiftTest, PeakJerkIsReportedAfterLandingInSettingUnits)
{
    armAndStage();
    setSC(SC_HIGH);
    acc.jerkMagnitude = 120.0f;
    advanceMs(100);
    acc.jerkMagnitude = 371.0f;      // the hardest moment of the climb
    advanceMs(1);
    acc.jerkMagnitude = 200.0f;
    advanceMs(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftImpacted());
    acc.jerkMagnitude = 0.0f;

    // land and disarm, switch still up: the number to set launch_lift_impact above
    disarmNow();
    EXPECT_STREQ("LIFT USED PEAK 38: SWITCH OFF", getLaunchControlLiftPreArmMessage());

    // off the scale: says so rather than wrapping
    setSC(SC_DOWN);
    setSC(SC_MID);
    arm();
    setSC(SC_HIGH);
    acc.jerkMagnitude = 9999.0f;
    advanceMs(LIFT_TIME_MS);
    acc.jerkMagnitude = 0.0f;
    disarmNow();
    EXPECT_STREQ("LIFT USED PEAK 250+: SWITCH OFF", getLaunchControlLiftPreArmMessage());
}

TEST(LaunchControlLiftTest, StickTakeoffDoesNotReportAPeak)
{
    armAndStage();
    rcData[THROTTLE] = STICK_TAKEOFF;   // took off on the sticks: the lift was skipped
    advanceMs(20);
    rcData[THROTTLE] = STICK_LOW;
    disarmNow();
    EXPECT_STREQ("LIFT USED: SWITCH OFF", getLaunchControlLiftPreArmMessage());
}

TEST(LaunchControlLiftTest, ALiftIsATakeoffSoTheSticksAreLiveAtHandover)
{
    // airmode on (feature or switch), default 25% start threshold, and the pilot never
    // touches the throttle: the lift itself counts as the takeoff
    armAndStage();
    simulationFeatureFlags |= FEATURE_AIRMODE;
    advanceMs(20);
    EXPECT_FALSE(wasThrottleRaised());               // on the ground: PIDs held off, no wind-up

    setSC(SC_HIGH);
    advanceMs(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(wasThrottleRaised());
    advanceMs(20);
    // stick still at zero, and the quad is flying on it: airmode descent, PIDs and I-term live
    EXPECT_EQ(PID_STABILISATION_ON, simulationPidStabilisationState);
    simulationFeatureFlags &= ~FEATURE_AIRMODE;
}

// STUBS
// ---------------------------------------------------------------------------
// launch_lift_brake: time up, throttle off, roll upside down, brake until the climb has
// stopped, then the sticks are live
// ---------------------------------------------------------------------------

#define LIFT_CLIMB_G 8.0f    // 7 g of climb on top of gravity: about 69 m/s after the 1 s lift

// the flight controller sees every loop, not just one long step
static void advanceMsInSteps(uint32_t ms)
{
    for (uint32_t done = 0; done < ms; done += 10) {
        advanceMs(MIN(10u, ms - done));
    }
}

// staged with the brake on, sent, and climbing hard to the end of the lift time
static void sendItWithTheBrake(uint8_t brake)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = brake;
    advanceMs(500);              // sat still, staged: learns what still reads
    setSC(SC_HIGH);              // sent
    setVerticalG(LIFT_CLIMB_G);
    advanceMs(LIFT_TIME_MS - 20);
}

TEST(LaunchControlLiftBrakeTest, TimeUpRollsItUpsideDownWithTheThrottleOff)
{
    sendItWithTheBrake(70);
    EXPECT_FALSE(isLaunchControlLiftBraking());
    EXPECT_FLOAT_EQ(1.0f, getLaunchControlLiftThrottle());
    EXPECT_NEAR(LIFT_CLIMB_G - 1.0f, getLaunchControlLiftClimbSpeed() / (G_ACCELERATION * (LIFT_TIME_MS - 20) / 1000.0f), 0.05f);

    advanceMs(20);   // time's up
    EXPECT_TRUE(isLaunchControlLifting());          // the pilot doesn't have it yet
    EXPECT_TRUE(isLaunchControlLiftBraking());      // OSD: LIFT BRAKE
    EXPECT_FALSE(isLaunchControlLiftHandoverRecent());
    EXPECT_EQ(0u, getLaunchControlLiftRemainingMs());
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftThrottle());   // throttle off for the flip
    EXPECT_FLOAT_EQ(LAUNCH_CONTROL_LIFT_FLIP_RATE_DEFAULT_DPS, getLaunchControlLiftRate(FD_ROLL));   // rolling over, flat out
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_PITCH));
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_YAW));
}

TEST(LaunchControlLiftBrakeTest, UpsideDownItBrakesUntilTheClimbStopsThenTheSticksAreLive)
{
    sendItWithTheBrake(70);
    advanceMs(20);                     // flipping
    const float speedAtTheEnd = getLaunchControlLiftClimbSpeed();
    EXPECT_GT(speedAtTheEnd, 60.0f);

    setAttitude(true);                 // over
    setVerticalG(-8.0f);               // thrust now pushes down: 9 g of stopping power
    advanceMs(20);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    EXPECT_FLOAT_EQ(0.7f, getLaunchControlLiftThrottle());
    EXPECT_NEAR(0.0f, getLaunchControlLiftRate(FD_ROLL), 1e-3f);   // holds it there
    EXPECT_NEAR(0.0f, getLaunchControlLiftRate(FD_PITCH), 1e-3f);

    // still going up for about 0.75 s more
    const uint32_t stopMs = (uint32_t)(speedAtTheEnd / (9.0f * G_ACCELERATION) * 1000);
    advanceMs(stopMs - 60);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    EXPECT_GT(getLaunchControlLiftClimbSpeed(), 0.0f);

    advanceMs(60);                     // stopped: the top of the jump
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftBraking());
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());   // STICKS LIVE
    EXPECT_FALSE(isLaunchControlLiftAborted());
    EXPECT_TRUE(ARMING_FLAG(ARMED));                    // never a disarm
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_ROLL));
}

TEST(LaunchControlLiftBrakeTest, SettingZeroHandsOverAtTimeUpAsBefore)
{
    sendItWithTheBrake(0);
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftBraking());
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
}

TEST(LaunchControlLiftBrakeTest, NotClimbingAtTimeUpMeansNoFlip)
{
    // tethered, or a lift throttle too low to leave the ground: nothing to brake,
    // and a flip a few centimetres up would only put it on its back
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = 70;
    advanceMs(500);
    setSC(SC_HIGH);
    setVerticalG(1.2f);   // barely moving: about 2 m/s after a second
    advanceMs(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftBrakeTest, TheSwitchAlwaysGivesThePilotTheSticks)
{
    sendItWithTheBrake(70);
    advanceMs(20);                     // flipping
    setSC(SC_MID);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());

    sendItWithTheBrake(70);
    advanceMs(20);
    setAttitude(true);
    setVerticalG(-8.0f);
    advanceMs(100);                    // braking
    EXPECT_TRUE(isLaunchControlLiftBraking());
    setSC(SC_DOWN);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftBrakeTest, FailsafeOrDisarmEndsTheBrake)
{
    sendItWithTheBrake(70);
    advanceMs(20);
    simulationFailsafeActive = true;
    advanceMs(1);
    EXPECT_FALSE(isLaunchControlLifting());
    simulationFailsafeActive = false;

    sendItWithTheBrake(70);
    advanceMs(20);
    disarmNow();
    EXPECT_FALSE(isLaunchControlLiftBraking());
}

TEST(LaunchControlLiftBrakeTest, AFlipThatDoesNotHappenHandsBackTheSticks)
{
    sendItWithTheBrake(70);
    advanceMs(20);                     // told to roll, but it stays upright (a dead motor, say)
    advanceMs(LAUNCH_CONTROL_LIFT_FLIP_TIMEOUT_MS + 300 - 20);   // 300 ms for the half turn at 1200 deg/s, twice
    EXPECT_TRUE(isLaunchControlLiftBraking());
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftAborted());          // LIFT ABORT: STICKS LIVE
    EXPECT_TRUE(isLaunchControlLiftHandoverRecent());
}

TEST(LaunchControlLiftBrakeTest, KnockedOffUpsideDownWhileBrakingHandsBackTheSticks)
{
    sendItWithTheBrake(70);
    advanceMs(20);
    setAttitude(true);
    setVerticalG(-8.0f);
    advanceMs(20);                     // braking
    setAttitude(false);                // tumbled back over
    setVerticalG(1.0f);
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS - 20);
    EXPECT_TRUE(isLaunchControlLiftBraking());          // a moment is ridden out
    advanceMsInSteps(40);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftBrakeTest, TheBrakeNeverRunsPastItsLimit)
{
    sendItWithTheBrake(70);
    advanceMs(20);
    setAttitude(true);
    setVerticalG(1.0f);                // the speed estimate never comes down (a bad accelerometer)
    advanceMs(20);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    advanceMs(LAUNCH_CONTROL_LIFT_BRAKE_TIME_DEFAULT_MS - 20);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlLifting());
}

TEST(LaunchControlLiftBrakeTest, TheBrakeJoltIsNotAnImpact)
{
    // the throttle step at the start of the brake is a big jolt by design
    sendItWithTheBrake(70);
    liftTestPidProfile.launchControlLiftImpact = 100;   // only read at the trigger: set for the next send
    sendItWithTheBrake(70);
    liftTestPidProfile.launchControlLiftImpact = 100;
    advanceMs(20);
    setAttitude(true);
    setVerticalG(-8.0f);
    acc.jerkMagnitude = 5000.0f;
    advanceMs(20);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    EXPECT_FALSE(isLaunchControlLiftImpacted());
    acc.jerkMagnitude = 0.0f;
}

TEST(LaunchControlLiftBrakeTest, AQuadThatCantHoldLevelInTheClimbIsNotFlipped)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = 70;
    setSC(SC_HIGH);
    setVerticalG(LIFT_CLIMB_G);
    advanceMs(300);                    // climbing fast, then a prop lets go
    mockCosTiltAngle = cos_approx(DEGREES_TO_RADIANS(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10));
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 20);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftBraking());
    EXPECT_TRUE(isLaunchControlLiftAborted());
    mockCosTiltAngle = 1.0f;
}

TEST(LaunchControlLiftBrakeTest, RollsTowardALeanAndSteersBackWhenPastIt)
{
    sendItWithTheBrake(70);
    // leaning a little left at time up: roll over to the left
    rMat.m[2][1] = -0.05f;
    rMat.m[2][2] = 0.9987f;
    advanceMs(20);
    EXPECT_FLOAT_EQ(-LAUNCH_CONTROL_LIFT_FLIP_RATE_DEFAULT_DPS, getLaunchControlLiftRate(FD_ROLL));

    // most of the way over, 20 deg short: slowing into it, same direction
    rMat.m[2][1] = -sinf(DEGREES_TO_RADIANS(160));
    rMat.m[2][2] = cosf(DEGREES_TO_RADIANS(160));
    advanceMs(1);
    EXPECT_NEAR(-LAUNCH_CONTROL_LIFT_FLIP_GAIN * 20, getLaunchControlLiftRate(FD_ROLL), 1.0f);

    // overshot by 10 deg: steers back
    rMat.m[2][1] = sinf(DEGREES_TO_RADIANS(170));
    rMat.m[2][2] = cosf(DEGREES_TO_RADIANS(170));
    advanceMs(1);
    EXPECT_NEAR(LAUNCH_CONTROL_LIFT_FLIP_GAIN * 10, getLaunchControlLiftRate(FD_ROLL), 1.0f);

    // nose 10 deg off upside down: pitch only
    rMat.m[2][0] = -sinf(DEGREES_TO_RADIANS(10));
    rMat.m[2][1] = 0.0f;
    rMat.m[2][2] = -cosf(DEGREES_TO_RADIANS(10));
    advanceMs(1);
    EXPECT_NEAR(0.0f, getLaunchControlLiftRate(FD_ROLL), 1e-3f);
    EXPECT_NEAR(LAUNCH_CONTROL_LIFT_FLIP_GAIN * 10, fabsf(getLaunchControlLiftRate(FD_PITCH)), 1.0f);
}

// ---------------------------------------------------------------------------
// Motor health: holding level isn't enough, every motor has to pull its weight
// ---------------------------------------------------------------------------

// motor outputs in DShot values (48..2047), as the mixer leaves them
static void setMotors(float m0, float m1, float m2, float m3)
{
    motor[0] = m0; motor[1] = m1; motor[2] = m2; motor[3] = m3;
}

static void setRpm(float r0, float r1, float r2, float r3)
{
    mockRpm[0] = r0; mockRpm[1] = r1; mockRpm[2] = r2; mockRpm[3] = r3;
}

static void sendItClimbing(uint8_t brake)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = brake;
    advanceMs(500);
    setSC(SC_HIGH);
    setVerticalG(LIFT_CLIMB_G);
}

TEST(LaunchControlLiftMotorTest, AMotorNotPullingItsWeightEndsTheLiftWithoutAFlip)
{
    sendItClimbing(70);
    setMotors(2000, 1950, 2047, 1900);          // a healthy, slightly tail-heavy climb
    advanceMsInSteps(400);
    EXPECT_TRUE(isLaunchControlLifting());

    setMotors(1500, 1500, 2047, 1450);          // rear-left pinned, the rest backed right off
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_MOTOR_FAULT_MS - 30);
    EXPECT_TRUE(isLaunchControlLifting());      // not yet: it has to last
    advanceMsInSteps(100);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftBraking()); // no flip for a hurt quad
    EXPECT_TRUE(isLaunchControlLiftAborted());
    EXPECT_STREQ("LIFT MOTOR: STICKS LIVE", getLaunchControlLiftHandoverMessage());
    EXPECT_TRUE(ARMING_FLAG(ARMED));
}

TEST(LaunchControlLiftMotorTest, AHealthyTailHeavyClimbIsFine)
{
    sendItClimbing(0);
    setMotors(2010, 1967, 2047, 1844);          // log 15, 10 Oct: about 10 % spread, all lift
    advanceMsInSteps(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_FALSE(isLaunchControlLiftAborted());
    EXPECT_STREQ("STICKS LIVE", getLaunchControlLiftHandoverMessage());
}

TEST(LaunchControlLiftMotorTest, AMomentaryImbalanceIsRiddenOut)
{
    sendItClimbing(0);
    setMotors(2000, 2000, 2000, 2000);
    advanceMsInSteps(400);
    setMotors(1400, 1400, 2047, 1400);          // a gust, a bump
    advanceMsInSteps(100);
    setMotors(2000, 2000, 2000, 2000);
    advanceMsInSteps(LIFT_TIME_MS - 500);
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftMotorTest, TheSpinUpIsNotJudged)
{
    sendItClimbing(0);
    setMotors(1200, 1300, 2047, 1100);          // leaving the ground unevenly
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_MOTOR_SETTLE_MS - 20);
    setMotors(2000, 2000, 2000, 2000);
    advanceMsInSteps(LIFT_TIME_MS - LAUNCH_CONTROL_LIFT_MOTOR_SETTLE_MS);
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftMotorTest, APropSpinningTooFastForItsCommandEndsTheLift)
{
    // a loose prop slipping on the shaft: the motor unloads and spins up, the quad still
    // holds level and the commands look fine
    sendItClimbing(70);
    useDshotTelemetry = true;
    mockRpmTelemetry = true;
    setMotors(1950, 1950, 1950, 1950);
    setRpm(30000, 30200, 29800, 30000);
    advanceMsInSteps(400);
    EXPECT_TRUE(isLaunchControlLifting());

    setRpm(30000, 30200, 34500, 30000);          // rear-left 15 % fast
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_MOTOR_FAULT_MS + 60);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_STREQ("LIFT MOTOR: STICKS LIVE", getLaunchControlLiftHandoverMessage());
}

TEST(LaunchControlLiftMotorTest, RpmIsOnlyJudgedWithRpmTelemetry)
{
    sendItClimbing(0);
    useDshotTelemetry = true;
    mockRpmTelemetry = false;                    // an ESC not reporting
    setMotors(1950, 1950, 1950, 1950);
    setRpm(30000, 30200, 45000, 30000);
    advanceMsInSteps(LIFT_TIME_MS);
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftMotorTest, AMotorNotTurningRefusesTheTrigger)
{
    // log 12301, 13 Oct: motor 2 held by grass at start-up while the other three idled.
    // Sent anyway, it tipped straight over.
    armAndStage();
    useDshotTelemetry = true;
    mockRpmTelemetry = true;
    setRpm(2000, 200, 2100, 1900);
    advanceMs(20);
    EXPECT_EQ(2, getLaunchControlLiftStoppedMotor());   // "LIFT: MOTOR 2 STOPPED"
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlActive());               // still staged, holding on the ground

    // freed with the trigger still up: must not launch by surprise
    setRpm(2000, 1950, 2100, 1900);
    advanceMs(200);
    EXPECT_EQ(0, getLaunchControlLiftStoppedMotor());
    EXPECT_FALSE(isLaunchControlLifting());

    // a fresh press sends it
    setSC(SC_MID);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftMotorTest, NoMotorTurningRefusesTheTrigger)
{
    // log 12302, 13 Oct: none of the four started, and a full-throttle lift moved nothing
    armAndStage();
    useDshotTelemetry = true;
    mockRpmTelemetry = true;
    setRpm(214, 214, 186, 214);                         // what a stopped motor reports on that ESC
    advanceMs(20);
    EXPECT_EQ(1, getLaunchControlLiftStoppedMotor());
    setSC(SC_HIGH);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_TRUE(isLaunchControlActive());
}

TEST(LaunchControlLiftMotorTest, WithoutRpmTelemetryTheTriggerIsNotHeldBack)
{
    armAndStage();
    setRpm(0, 0, 0, 0);
    useDshotTelemetry = true;
    mockRpmTelemetry = false;                           // no RPM to judge by
    advanceMs(20);
    EXPECT_EQ(0, getLaunchControlLiftStoppedMotor());
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
}

TEST(LaunchControlLiftMotorTest, AMotorStoppingInTheClimbEndsTheLiftAtOnce)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = 70;
    useDshotTelemetry = true;
    mockRpmTelemetry = true;
    setRpm(2000, 2000, 2000, 2000);                     // idling
    advanceMs(500);
    setSC(SC_HIGH);
    EXPECT_TRUE(isLaunchControlLifting());
    setVerticalG(LIFT_CLIMB_G);
    setMotors(1950, 1950, 1950, 1950);
    setRpm(30000, 30000, 30000, 30000);
    advanceMsInSteps(100);

    setRpm(30000, 30000, 0, 30000);                     // an ESC resets
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_MOTOR_STOPPED_MS - 30);
    EXPECT_TRUE(isLaunchControlLifting());              // a moment's dropout is ridden out
    advanceMsInSteps(60);
    EXPECT_FALSE(isLaunchControlLifting());             // inside the settle time the other checks wait out
    EXPECT_FALSE(isLaunchControlLiftBraking());         // no flip for a hurt quad
    EXPECT_STREQ("LIFT MOTOR: STICKS LIVE", getLaunchControlLiftHandoverMessage());
    EXPECT_TRUE(ARMING_FLAG(ARMED));
}

// ---------------------------------------------------------------------------
// How high it went: the OSD's "LAUNCH: 272'"
// ---------------------------------------------------------------------------

TEST(LaunchControlLiftHeightTest, ItShowsTheClimbThenTheTopOfTheCoast)
{
    armAndStage();
    advanceMs(500);                                     // sat still, staged
    bool atTop = true;
    EXPECT_EQ(-1, getLaunchControlLiftHeightCm(&atTop)); // nothing to show yet
    EXPECT_FALSE(atTop);

    setSC(SC_HIGH);                                     // sent
    setVerticalG(3.0f);                                 // 2 g of climb
    advanceMsInSteps(500);
    EXPECT_NEAR(245, getLaunchControlLiftHeightCm(&atTop), 15);   // 1/2 x 2 g x 0.5 s squared
    EXPECT_FALSE(atTop);                                // still going: "LAUNCH ^8'"

    advanceMsInSteps(LIFT_TIME_MS - 500);               // time up, no brake: handed over at about 20 m/s
    EXPECT_FALSE(isLaunchControlLifting());
    setVerticalG(0.0f);                                 // coasting up on its own
    advanceMsInSteps(1000);
    EXPECT_FALSE(isLaunchControlLifting());
    getLaunchControlLiftHeightCm(&atTop);
    EXPECT_FALSE(atTop);                                // still rising: followed after the handover

    advanceMsInSteps(1200);                             // the coast has topped out
    EXPECT_NEAR(2940, getLaunchControlLiftHeightCm(&atTop), 60);  // 9.8 m in the lift + 19.6 m of coast
    EXPECT_TRUE(atTop);                                 // final: "LAUNCH: 96'"

    setVerticalG(1.0f);                                 // flying on
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_TOP_NOTICE_MS - 1500);
    EXPECT_NEAR(2940, getLaunchControlLiftHeightCm(&atTop), 60);  // the top stays the top
    advanceMsInSteps(2000);
    EXPECT_EQ(-1, getLaunchControlLiftHeightCm(&atTop)); // and is gone after a while
}

TEST(LaunchControlLiftHeightTest, ABrakeStopHandsOverAtTheTop)
{
    sendItWithTheBrake(70);
    advanceMs(20);                                      // flipping
    setAttitude(true);
    setVerticalG(-8.0f);                                // braking
    bool atTop = false;
    while (isLaunchControlLifting()) {
        advanceMs(10);
        EXPECT_GE(getLaunchControlLiftHeightCm(&atTop), 0);
    }
    EXPECT_GT(getLaunchControlLiftHeightCm(&atTop), 0);
    EXPECT_TRUE(atTop);                                 // stopped is the top: final at once
}

TEST(LaunchControlLiftHeightTest, TheTopDoesNotComeBackAfterTheTimerWraps)
{
    sendItWithTheBrake(70);
    advanceMs(20);                                      // flipping
    setAttitude(true);
    setVerticalG(-8.0f);                                // braking
    while (isLaunchControlLifting()) {
        advanceMs(10);
    }
    bool atTop = false;
    EXPECT_GT(getLaunchControlLiftHeightCm(&atTop), 0);
    EXPECT_TRUE(atTop);
    setVerticalG(1.0f);                                 // flying on
    advanceMs(LAUNCH_CONTROL_LIFT_TOP_NOTICE_MS);
    EXPECT_EQ(-1, getLaunchControlLiftHeightCm(&atTop));

    // still flying when the 32-bit microsecond timer has gone all the way round (about 71.6 minutes)
    for (int minutes = 1; minutes <= 71; minutes++) {
        advanceMs(60 * 1000);
    }
    for (int step = 0; step < 400; step++) {
        advanceMs(100);
        ASSERT_EQ(-1, getLaunchControlLiftHeightCm(&atTop)) << "returned " << step * 100 << " ms after 71 minutes";
    }
}

TEST(LaunchControlLiftHeightTest, ADisarmClearsIt)
{
    sendItWithTheBrake(0);
    advanceMs(20);                                      // handed over
    disarmNow();
    bool atTop;
    EXPECT_EQ(-1, getLaunchControlLiftHeightCm(&atTop));
    arm();
    EXPECT_EQ(-1, getLaunchControlLiftHeightCm(&atTop)); // a new flight starts blank
}

TEST(LaunchControlLiftMotorTest, TheBrakeIsNotJudged)
{
    // rolling over and braking drives the motors very unevenly by design
    sendItWithTheBrake(70);
    setMotors(2000, 2000, 2000, 2000);
    advanceMs(20);                               // flipping
    setMotors(300, 300, 2047, 2047);
    advanceMsInSteps(300);
    EXPECT_FALSE(isLaunchControlLiftAborted());
}

TEST(LaunchControlLiftMessageTest, TheOsdSaysWhyTheLiftEnded)
{
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(LIFT_TIME_MS);
    EXPECT_STREQ("STICKS LIVE", getLaunchControlLiftHandoverMessage());

    armAndStage();
    setSC(SC_HIGH);
    advanceMs(200);
    setSC(SC_MID);                               // switched back
    EXPECT_STREQ("LIFT ABORT: STICKS LIVE", getLaunchControlLiftHandoverMessage());
    EXPECT_FALSE(isLaunchControlLiftAborted());  // the pilot's call, not a fault

    armAndStage();
    setSC(SC_HIGH);
    mockCosTiltAngle = cos_approx(DEGREES_TO_RADIANS(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10));
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 20);
    mockCosTiltAngle = 1.0f;
    EXPECT_STREQ("LIFT FAULT: STICKS LIVE", getLaunchControlLiftHandoverMessage());

    armAndStage();
    liftTestPidProfile.launchControlLiftImpact = 100;
    setSC(SC_HIGH);
    acc.jerkMagnitude = 5000.0f;
    advanceMs(1);
    acc.jerkMagnitude = 0.0f;
    EXPECT_STREQ("LIFT IMPACT: STICKS LIVE", getLaunchControlLiftHandoverMessage());
}

// ---------------------------------------------------------------------------
// The flip is configurable: direction, rate, how long the brake may run
// ---------------------------------------------------------------------------

// sent with the brake on, lift time up, rolling over: the first rate the flip asks for
static void timeUpWith(uint8_t direction, uint16_t rate)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = 70;
    liftTestPidProfile.launchControlLiftFlipDirection = direction;
    liftTestPidProfile.launchControlLiftFlipRate = rate;
    advanceMs(500);
    setSC(SC_HIGH);
    setVerticalG(LIFT_CLIMB_G);
    advanceMs(LIFT_TIME_MS);
    ASSERT_TRUE(isLaunchControlLiftBraking());
}

TEST(LaunchControlLiftFlipTest, EachDirectionTurnsTheWayTheSticksWould)
{
    timeUpWith(LAUNCH_LIFT_FLIP_RIGHT, 1000);
    EXPECT_FLOAT_EQ(1000.0f, getLaunchControlLiftRate(FD_ROLL));
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_PITCH));

    timeUpWith(LAUNCH_LIFT_FLIP_LEFT, 1000);
    EXPECT_FLOAT_EQ(-1000.0f, getLaunchControlLiftRate(FD_ROLL));

    timeUpWith(LAUNCH_LIFT_FLIP_FORWARD, 1000);
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_ROLL));
    EXPECT_FLOAT_EQ(1000.0f, getLaunchControlLiftRate(FD_PITCH));

    timeUpWith(LAUNCH_LIFT_FLIP_BACK, 1000);
    EXPECT_FLOAT_EQ(-1000.0f, getLaunchControlLiftRate(FD_PITCH));
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_YAW));
}

TEST(LaunchControlLiftFlipTest, AChosenDirectionIgnoresTheLean)
{
    timeUpWith(LAUNCH_LIFT_FLIP_RIGHT, 1200);
    // leaning 20 deg left: AUTO would go left, RIGHT still goes right
    rMat.m[2][1] = -sinf(DEGREES_TO_RADIANS(20));
    rMat.m[2][2] = cosf(DEGREES_TO_RADIANS(20));
    advanceMs(1);
    EXPECT_FLOAT_EQ(1200.0f, getLaunchControlLiftRate(FD_ROLL));

    // past sideways (rolled 120 deg right), the usual steering finishes it: still right
    rMat.m[2][1] = sinf(DEGREES_TO_RADIANS(120));
    rMat.m[2][2] = cosf(DEGREES_TO_RADIANS(120));
    advanceMs(1);
    EXPECT_NEAR(LAUNCH_CONTROL_LIFT_FLIP_GAIN * 60, getLaunchControlLiftRate(FD_ROLL), 1.0f);   // 60 deg to go, slowing into it
}

TEST(LaunchControlLiftFlipTest, TheFlipRateCapsTheTurn)
{
    timeUpWith(LAUNCH_LIFT_FLIP_AUTO, 600);
    EXPECT_FLOAT_EQ(600.0f, getLaunchControlLiftRate(FD_ROLL));
    // out of range settings are held to the limits
    timeUpWith(LAUNCH_LIFT_FLIP_AUTO, 5000);
    EXPECT_FLOAT_EQ(LAUNCH_CONTROL_LIFT_FLIP_RATE_MAX_DPS, getLaunchControlLiftRate(FD_ROLL));
    timeUpWith(LAUNCH_LIFT_FLIP_AUTO, 0);
    EXPECT_FLOAT_EQ(LAUNCH_CONTROL_LIFT_FLIP_RATE_MIN_DPS, getLaunchControlLiftRate(FD_ROLL));
}

TEST(LaunchControlLiftFlipTest, ASlowFlipGetsLongerBeforeItCountsAsFailed)
{
    timeUpWith(LAUNCH_LIFT_FLIP_AUTO, 300);   // a half turn takes 0.6 s: 300 + 1200 ms allowed
    advanceMsInSteps(1400);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    advanceMsInSteps(200);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_STREQ("LIFT FAULT: STICKS LIVE", getLaunchControlLiftHandoverMessage());
}

TEST(LaunchControlLiftFlipTest, TheBrakeTimeSettingLimitsTheBrake)
{
    armAndStage();
    liftTestPidProfile.launchControlLiftBrake = 70;
    liftTestPidProfile.launchControlLiftBrakeTime = 400;
    advanceMs(500);
    setSC(SC_HIGH);
    setVerticalG(LIFT_CLIMB_G);
    advanceMs(LIFT_TIME_MS);
    setAttitude(true);
    setVerticalG(1.0f);                // the speed estimate never comes down
    advanceMs(20);                     // braking
    advanceMs(380);
    EXPECT_TRUE(isLaunchControlLiftBraking());
    advanceMs(20);
    EXPECT_FALSE(isLaunchControlLifting());
}

TEST(LaunchControlLiftFlipTest, ItHandsOverUpsideDownOnceTheClimbHasStopped)
{
    // the climb stopped, still upside down: sticks live, throttle back to the stick
    // (zero here), so it falls in airmode until the pilot rolls it back over
    timeUpWith(LAUNCH_LIFT_FLIP_RIGHT, 1200);
    setAttitude(true);
    setVerticalG(-8.0f);
    advanceMsInSteps(1000);
    EXPECT_FALSE(isLaunchControlLifting());
    EXPECT_STREQ("STICKS LIVE", getLaunchControlLiftHandoverMessage());
    EXPECT_LE(getLaunchControlLiftClimbSpeed(), 0.0f);
    EXPECT_FLOAT_EQ(0.0f, getLaunchControlLiftRate(FD_ROLL));   // nothing steering it any more
}

// ---------------------------------------------------------------------------
// The blackbox log records each step of a lift, and why it ended
// ---------------------------------------------------------------------------

TEST(LaunchControlLiftLogTest, EachStepOfALiftWithTheBrake)
{
    setUpLift();
    EXPECT_EQ(LAUNCH_LIFT_LOG_NONE, getLaunchControlLiftLogState());
    setSC(SC_MID);
    arm();
    EXPECT_EQ(LAUNCH_LIFT_LOG_STAGED, getLaunchControlLiftLogState());
    liftTestPidProfile.launchControlLiftBrake = 70;
    advanceMs(500);

    setSC(SC_HIGH);
    EXPECT_EQ(LAUNCH_LIFT_LOG_CLIMB, getLaunchControlLiftLogState());
    setVerticalG(LIFT_CLIMB_G);
    advanceMs(LIFT_TIME_MS - 20);
    advanceMs(20);
    EXPECT_EQ(LAUNCH_LIFT_LOG_FLIP, getLaunchControlLiftLogState());
    setAttitude(true);
    setVerticalG(-8.0f);
    advanceMs(20);
    EXPECT_EQ(LAUNCH_LIFT_LOG_BRAKE, getLaunchControlLiftLogState());
    advanceMsInSteps(1000);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_STOPPED, getLaunchControlLiftLogState());

    disarmNow();   // the outcome stays readable for the rest of the log
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_STOPPED, getLaunchControlLiftLogState());
}

TEST(LaunchControlLiftLogTest, HowEachLiftEndedIsTold)
{
    // no brake: time up
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(LIFT_TIME_MS);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_TIME, getLaunchControlLiftLogState());

    // the pilot's switch
    armAndStage();
    setSC(SC_HIGH);
    advanceMs(100);
    setSC(SC_MID);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_SWITCH, getLaunchControlLiftLogState());

    // failsafe
    armAndStage();
    setSC(SC_HIGH);
    simulationFailsafeActive = true;
    advanceMs(1);
    simulationFailsafeActive = false;
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_FAILSAFE, getLaunchControlLiftLogState());

    // disarmed mid-lift
    armAndStage();
    setSC(SC_HIGH);
    disarmNow();
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_DISARM, getLaunchControlLiftLogState());

    // couldn't hold level
    armAndStage();
    setSC(SC_HIGH);
    mockCosTiltAngle = cos_approx(DEGREES_TO_RADIANS(LAUNCH_CONTROL_LIFT_ABORT_TILT_DEG + 10));
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 20);
    mockCosTiltAngle = 1.0f;
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_CONTROL, getLaunchControlLiftLogState());

    // a motor
    sendItClimbing(0);
    setMotors(2000, 2000, 2000, 2000);
    advanceMsInSteps(400);
    setMotors(1450, 1450, 2047, 1450);
    advanceMsInSteps(300);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_MOTOR, getLaunchControlLiftLogState());

    // an impact
    armAndStage();
    liftTestPidProfile.launchControlLiftImpact = 100;
    setSC(SC_HIGH);
    acc.jerkMagnitude = 5000.0f;
    advanceMs(1);
    acc.jerkMagnitude = 0.0f;
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_IMPACT, getLaunchControlLiftLogState());
}

TEST(LaunchControlLiftLogTest, HowABrakeEndedIsTold)
{
    // the flip didn't happen
    sendItWithTheBrake(70);
    advanceMsInSteps(1000);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_FLIP, getLaunchControlLiftLogState());

    // knocked off upside down while braking
    sendItWithTheBrake(70);
    advanceMs(20);
    setAttitude(true);
    setVerticalG(-8.0f);
    advanceMs(20);
    setAttitude(false);
    setVerticalG(1.0f);
    advanceMsInSteps(LAUNCH_CONTROL_LIFT_ABORT_TIME_MS + 40);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_UPSET, getLaunchControlLiftLogState());

    // brake time ran out before the climb stopped
    sendItWithTheBrake(70);
    advanceMs(20);
    setAttitude(true);
    setVerticalG(1.0f);
    advanceMs(20);
    advanceMs(LAUNCH_CONTROL_LIFT_BRAKE_TIME_DEFAULT_MS);
    EXPECT_EQ(LAUNCH_LIFT_LOG_ENDED + LIFT_END_BRAKE_LIMIT, getLaunchControlLiftLogState());
}

TEST(LaunchControlLiftLogTest, ARegularArmLogsNothing)
{
    setUpLift();
    arm();                       // SC down: regular flight
    EXPECT_EQ(LAUNCH_LIFT_LOG_NONE, getLaunchControlLiftLogState());
}

extern "C" {
    void sincosf_approx(float x, float *out_s, float *out_c) {
        *out_s = sin_approx(x);
        *out_c = cos_approx(x);
    }

    uint32_t micros(void) { return simulationTime; }
    uint32_t millis(void) { return micros() / 1000; }
    bool isRxReceivingSignal(void) { return simulationHaveRx; }

    bool featureIsEnabled(uint32_t f) { return simulationFeatureFlags & f; }
    void warningLedFlash(void) {}
    void warningLedDisable(void) {}
    void warningLedUpdate(void) {}
    void beeper(beeperMode_e) {}
    void beeperConfirmationBeeps(uint8_t) {}
    void tasksUpdateModeGatedEnables(void) {}
    void beeperWarningBeeps(uint8_t) {}
    void beeperSilence(void) {}
    void systemBeep(bool) {}
    void saveConfigAndNotify(void) {}
    void blackboxFinish(void) {}
    bool accIsCalibrationComplete(void) { return true; }
    bool baroIsCalibrated(void) { return true; }
    bool gyroIsCalibrationComplete(void) { return gyroCalibDone; }
    void gyroStartCalibration(bool) {}
    bool isFirstArmingGyroCalibrationRunning(void) { return false; }
    void pidController(const pidProfile_t *, timeUs_t) {}
    void pidStabilisationState(pidStabilisationState_e state) { simulationPidStabilisationState = state; }
    void mixTable(timeUs_t) {};
    void writeMotors(void) {};
    void writeServos(void) {};
    bool calculateRxChannelsAndUpdateFailsafe(timeUs_t) { return true; }
    bool isMixerUsingServos(void) { return false; }
    void gyroUpdate(void) {}
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
    void updateRSSI(timeUs_t) {}
    bool failsafeIsMonitoring(void) { return false; }
    void failsafeStartMonitoring(void) {}
    void failsafeUpdateState(void) {}
    bool failsafeIsActive(void) { return simulationFailsafeActive; }
    bool failsafeIsReceivingRxData(void) { return true; }
    bool rxAreFlightChannelsValid(void) { return false; }
    void pidResetIterm(void) {}
    void updateAdjustmentStates(void) {}
    void processRcAdjustments(controlRateConfig_t *) {}
    void updateGpsWaypointsAndMode(void) {}
    void mspSerialReleaseSharedTelemetryPorts(void) {}
    void telemetryCheckState(void) {}
    void mspSerialAllocatePorts(void) {}
    void gyroReadTemperature(void) {}
    void updateRcCommands(void) {}
    void applyAltHold(void) {}
    void resetYawAxis(void) {}
    int16_t calculateThrottleAngleCorrection(uint8_t) { return 0; }
    void processRcCommand(void) {}
    void updateGpsStateForHomeAndHoldMode(void) {}
    void blackboxUpdate(timeUs_t) {}
    void transponderUpdate(timeUs_t) {}
    void GPS_reset_home_position(void) {}
    void accStartCalibration(void) {}
    bool mockAccCalibrated = true;
    bool accHasBeenCalibrated(void) { return mockAccCalibrated; }
    void baroSetGroundLevel(void) {}
    void changePidProfile(uint8_t) {}
    void changeControlRateProfile(uint8_t) {}
    void dashboardEnablePageCycling(void) {}
    void dashboardDisablePageCycling(void) {}
    bool imuQuaternionHeadfreeOffsetSet(void) { return true; }
    void rescheduleTask(taskId_e, timeDelta_t) {}
    bool usbCableIsInserted(void) { return false; }
    bool usbVcpIsConnected(void) { return false; }
    void pidSetAntiGravityState(bool) {}
    void osdSuppressStats(bool) {}
    float scaleRangef(float, float, float, float, float) { return 0.0f; }
    bool crashRecoveryModeActive(void) { return false; }
    int32_t getEstimatedAltitudeCm(void) { return 0; }
    bool gpsIsHealthy(void) { return false; }
    float getCosTiltAngle(void) { return mockCosTiltAngle; }
    void pidSetItermReset(bool) {}
    void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
    bool isFixedWing(void) { return false; }
    void compassStartCalibration(void) {}
    bool compassIsCalibrationComplete(void) { return true; }
    bool compassEnabledAndCalibrated(void) { return true; }
    bool isUpright(void) { return mockIsUpright; }
    uint16_t gyroAbsRateDps(int) { return mockGyroAbsRateDps; }
    void blackboxLogEvent(FlightLogEvent, union flightLogEventData_u *) {};
    void gyroFiltering(timeUs_t) {};
    timeDelta_t rxGetFrameDelta() { return 0; }
    void updateRcRefreshRate(timeUs_t) {};
    uint16_t getAverageSystemLoadPercent(void) { return 0; }
    bool isMotorProtocolEnabled(void) { return true; }
    void pinioBoxTaskControl(void) {}
    void schedulerSetNextStateTime(timeDelta_t) {}

    bool isAltitudeAvailable(void)  { return true; }
    float getAltitudeCm(void) {return 0.0f;}
    float getAltitudeDerivative(void) {return 0.0f;}
    float getAltitudeCmControl(void) { return 0.0f; }
    float getAltitudeDerivativeControl(void) { return 0.0f; }
    float getAltitudeAccelerationControl(void) { return 0.0f; }


    float sin_approx(float x) { return sinf(x); }
    float cos_approx(float x) { return cosf(x); }
    float atan2_approx(float, float) {return 0.0f;}
    float acos_approx(float x) { return acosf(x); }
    uint8_t getMotorCount(void) { return 4; }
    float getMotorOutputLow(void) { return 48.0f; }
    float getMotorOutputHigh(void) { return 2047.0f; }
    bool isDshotTelemetryActive(void) { return mockRpmTelemetry; }
    float getDshotRpm(uint8_t motorIndex) { return mockRpm[motorIndex]; }
    void updateDshotTelemetry(void) {}

    void getRcDeflectionAbs(void) {}
    uint32_t getCpuPercentageLate(void) { return 0; }
    void GPS_distance_cm_bearing(const gpsLocation_t *from, const gpsLocation_t *to, bool dist3d, uint32_t *dist, int32_t *bearing)
    {
       UNUSED(from);
       UNUSED(to);
       UNUSED(dist3d);
       UNUSED(dist);
       UNUSED(bearing);
    }

void GPS_distance2d(const gpsLocation_t* /*from*/, const gpsLocation_t* /*to*/, vector2_t* /*dest*/) { }

    static positionEstimate3d_t stubEstimate = {};
    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &stubEstimate; }
    void positionEstimatorEnableXY(bool /*enable*/) { }
    bool positionEstimatorIsValidXY(void) { return false; }

    void positionNavInit(void) { }
    void positionNavReset(void) { }
    void positionNavUpdate(float /*dt*/, const positionEstimate3d_t * /*est*/) { }
    bool positionNavHasActiveTarget(void) { return false; }
    bool positionNavTargetReached(void) { return false; }
    vector3_t positionNavGetTargetVelocityCmS(void) { return (vector3_t){{0, 0, 0}}; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return NULL; }
    bool imuIsHeadingValid(void) { return true; }

    bool canUseGPSHeading;
    bool gpsHasNewData(uint16_t* gpsStamp) {
        UNUSED(*gpsStamp);
        return true;
    }
    float getSetpointRate(int axis)
    {
        UNUSED(axis);
        return 0.0f;
    }

    float getMaxRcRate(int axis)
    {
        UNUSED(axis);
        return 720.0f; // nonzero: autopilotInit divides maxVelocity by this
    }

    float getGpsCosLat(void) { return 1.0f; }
}
