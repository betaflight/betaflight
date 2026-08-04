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

extern "C" {
    #include "blackbox/blackbox.h"
    #include "build/debug.h"
    #include "common/maths.h"
    #include "common/streambuf.h"

    #include "config/feature.h"
    #include "config/config.h"

    #include "fc/controlrate_profile.h"
    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/servos.h"
    #include "flight/gps_rescue.h"

    #include "io/beeper.h"
    #include "io/gps.h"
    #include "io/vtx.h"
    #include "io/vtx_control.h"

    #include "drivers/vtx_common.h"

    #include "pg/gps_rescue.h"
    #include "pg/motor.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "rx/rx.h"

    #include "scheduler/scheduler.h"
    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"
    #include "telemetry/telemetry.h"

    vtxSettingsConfig_t vtxGetSettings(void);

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
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
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_CONFIG, 0);

    extern float rcCommand[4];
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint16_t averageSystemLoadPercent = 0;
    uint8_t cliMode = 0;
    uint8_t debugMode = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    pidProfile_t *currentPidProfile;
    controlRateConfig_t *currentControlRateProfile;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;
    uint32_t targetPidLooptime;
    bool cmsInMenu = false;
    float axisPID_P[3], axisPID_I[3], axisPID_D[3], axisPIDSum[3];
    rxRuntimeState_t rxRuntimeState = {};
    acc_t acc;
}

uint32_t simulationFeatureFlags = 0;
uint32_t simulationTime = 0;
bool gyroCalibDone = false;
bool simulationHaveRx = false;

#include "gtest/gtest.h"

TEST(VtxTest, PitMode)
{
    // given
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXVTXPITMODE;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1750);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MAX);

    analyzeModeActivationConditions();

    // and
    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->freq = 5800;
    vtxSettingsConfigMutable()->pitModeFreq = 5300;

    // expect
    EXPECT_EQ(5800, vtxGetSettings().freq);

    // and
    // enable vtx pit mode
    rcData[AUX1] = 1800;

    // when
    updateActivatedModes();

    // expect
    EXPECT_TRUE(IS_RC_MODE_ACTIVE(BOXVTXPITMODE));
    EXPECT_EQ(5300, vtxGetSettings().freq);
}

// vtxUpdateActivatedChannel() only checks that a device is present; it never calls into the
// vTable, so a stub device with no vTable is sufficient.
static vtxDevice_t testVtxDevice = { .vTable = NULL };

extern "C" {
    // STATIC_UNIT_TESTED in vtx_control.c — a session-sticky arm latch with no production reset.
    // Reset it before each case so the tests are independent of execution order (e.g. shuffle).
    extern uint8_t locked;
}

static void clearActivationConditions(void)
{
    for (int i = 0; i < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; i++) {
        vtxChannelActivationCondition_t *cond = &vtxConfigMutable()->vtxChannelActivationConditions[i];
        cond->auxChannelIndex = 0;
        cond->band = 0;
        cond->channel = 0;
        cond->power = 0;
        cond->range.startStep = 0;
        cond->range.endStep = 0;
    }
}

static void setActivationCondition(uint8_t index, uint8_t auxChannelIndex, uint8_t band, uint8_t channel, uint8_t power)
{
    vtxChannelActivationCondition_t *cond = &vtxConfigMutable()->vtxChannelActivationConditions[index];
    cond->auxChannelIndex = auxChannelIndex;
    cond->band = band;
    cond->channel = channel;
    cond->power = power;
    cond->range.startStep = CHANNEL_VALUE_TO_STEP(1400);
    cond->range.endStep = CHANNEL_VALUE_TO_STEP(1600);
    // drive the aux channel into the active range
    rcData[NON_AUX_CHANNEL_COUNT + auxChannelIndex] = 1500;
}

// Regression test for #15362: band, channel and power assigned to three separate switches,
// all active simultaneously. The power condition sits at the highest array index. Before the
// fix, only one condition was applied per call and the two lowest indices ping-ponged, so the
// highest-indexed (power) condition was never reached.
TEST(VtxTest, ActivationConditionsAllAppliedWhenSimultaneouslyActive)
{
    // given
    locked = 0;
    DISABLE_ARMING_FLAG(ARMED);
    vtxCommonSetDevice(&testVtxDevice);
    clearActivationConditions();

    setActivationCondition(0, 0, 1, 0, 0);  // AUX1 -> band 1
    setActivationCondition(1, 1, 0, 2, 0);  // AUX2 -> channel 2
    setActivationCondition(2, 2, 0, 0, 3);  // AUX3 -> power 3 (highest index)

    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = 0;
    vtxSettingsConfigMutable()->power = 0;

    // when
    vtxUpdateActivatedChannel();

    // expect — all three applied, in particular power (the previously starved condition)
    EXPECT_EQ(1, vtxSettingsConfig()->band);
    EXPECT_EQ(2, vtxSettingsConfig()->channel);
    EXPECT_EQ(3, vtxSettingsConfig()->power);
}

// A power-only condition (band == 0, channel == 0) must change only power, leaving band and
// channel untouched — the per-field `> 0` guards must survive removal of the early break.
TEST(VtxTest, PowerOnlyConditionDoesNotResetBandAndChannel)
{
    // given
    locked = 0;
    DISABLE_ARMING_FLAG(ARMED);
    vtxCommonSetDevice(&testVtxDevice);
    clearActivationConditions();

    setActivationCondition(0, 0, 0, 0, 2);  // AUX1 -> power 2 only

    vtxSettingsConfigMutable()->band = 5;
    vtxSettingsConfigMutable()->channel = 7;
    vtxSettingsConfigMutable()->power = 0;

    // when
    vtxUpdateActivatedChannel();

    // expect — power updated, band/channel preserved
    EXPECT_EQ(5, vtxSettingsConfig()->band);
    EXPECT_EQ(7, vtxSettingsConfig()->channel);
    EXPECT_EQ(2, vtxSettingsConfig()->power);
}

// Last-write-wins precedence: when two active conditions set the SAME field, the highest-indexed
// one wins because every active condition is applied in array order. This is the documented
// behaviour change introduced with the starvation fix.
TEST(VtxTest, LastActiveConditionWinsForSameField)
{
    // given
    locked = 0;
    DISABLE_ARMING_FLAG(ARMED);
    vtxCommonSetDevice(&testVtxDevice);
    clearActivationConditions();

    // two conditions, both active, both writing band/channel/power
    setActivationCondition(0, 0, 1, 1, 1);  // AUX1 -> band 1, channel 1, power 1
    setActivationCondition(1, 1, 5, 6, 7);  // AUX2 -> band 5, channel 6, power 7 (higher index)

    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = 0;
    vtxSettingsConfigMutable()->power = 0;

    // when
    vtxUpdateActivatedChannel();

    // expect — highest-indexed active condition wins every field
    EXPECT_EQ(5, vtxSettingsConfig()->band);
    EXPECT_EQ(6, vtxSettingsConfig()->channel);
    EXPECT_EQ(7, vtxSettingsConfig()->power);
}

// A condition becoming inactive (range no longer matched) is level-triggered with no reset:
// the last applied values must be left in place, i.e. it must NOT reset settings to 0.
TEST(VtxTest, InactiveConditionLeavesLastValues)
{
    // given
    locked = 0;
    DISABLE_ARMING_FLAG(ARMED);
    vtxCommonSetDevice(&testVtxDevice);
    clearActivationConditions();

    setActivationCondition(0, 0, 3, 4, 5);  // AUX1 -> band 3, channel 4, power 5, active

    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = 0;
    vtxSettingsConfigMutable()->power = 0;

    vtxUpdateActivatedChannel();
    EXPECT_EQ(3, vtxSettingsConfig()->band);
    EXPECT_EQ(4, vtxSettingsConfig()->channel);
    EXPECT_EQ(5, vtxSettingsConfig()->power);

    // when the aux channel leaves the active range
    rcData[NON_AUX_CHANNEL_COUNT + 0] = 1000;
    vtxUpdateActivatedChannel();

    // expect — values persist, nothing is reset
    EXPECT_EQ(3, vtxSettingsConfig()->band);
    EXPECT_EQ(4, vtxSettingsConfig()->channel);
    EXPECT_EQ(5, vtxSettingsConfig()->power);
}

// When armed, band/channel are locked but power may still change. `locked` is reset first so
// this case no longer depends on running last.
TEST(VtxTest, ArmedLocksBandAndChannelButNotPower)
{
    // given
    locked = 0;
    DISABLE_ARMING_FLAG(ARMED);
    vtxCommonSetDevice(&testVtxDevice);
    clearActivationConditions();

    setActivationCondition(0, 0, 4, 6, 2);  // AUX1 -> band 4, channel 6, power 2

    vtxSettingsConfigMutable()->band = 0;
    vtxSettingsConfigMutable()->channel = 0;
    vtxSettingsConfigMutable()->power = 0;

    // when armed
    ENABLE_ARMING_FLAG(ARMED);
    vtxUpdateActivatedChannel();
    DISABLE_ARMING_FLAG(ARMED);  // restore global state so later/shuffled tests don't inherit it

    // expect — band/channel locked out, power still applied
    EXPECT_EQ(0, vtxSettingsConfig()->band);
    EXPECT_EQ(0, vtxSettingsConfig()->channel);
    EXPECT_EQ(2, vtxSettingsConfig()->power);
}

// STUBS
extern "C" {
    uint8_t activePidLoopDenom = 1;
    uint32_t micros(void) { return simulationTime; }
    uint32_t millis(void) { return micros() / 1000; }
    bool isRxReceivingSignal(void) { return simulationHaveRx; }

    bool featureIsEnabled(uint32_t f) { return simulationFeatureFlags & f; }
    void warningLedFlash(void) {}
    void warningLedDisable(void) {}
    void warningLedUpdate(void) {}
    void beeper(beeperMode_e) {}
    void beeperConfirmationBeeps(uint8_t) {}
    void beeperWarningBeeps(uint8_t) {}
    void beeperSilence(void) {}
    void systemBeep(bool) {}
    void saveConfigAndNotify(void) {}
    void blackboxFinish(void) {}
    bool accIsCalibrationComplete(void) { return true; }
    bool accHasBeenCalibrated(void) { return true; }
    bool baroIsCalibrated(void) { return true; }
    bool gyroIsCalibrationComplete(void) { return gyroCalibDone; }
    void gyroStartCalibration(bool) {}
    bool isFirstArmingGyroCalibrationRunning(void) { return false; }
    void pidController(const pidProfile_t *, timeUs_t) {}
    void pidStabilisationState(pidStabilisationState_e) {}
    void mixTable(timeUs_t) {};
    void writeMotors(void) {};
    void writeServos(void) {};
    bool calculateRxChannelsAndUpdateFailsafe(timeUs_t) { return true; }
    bool isMixerUsingServos(void) { return false; }
    void gyroUpdate() {}
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
    void updateRSSI(timeUs_t) {}
    bool failsafeIsMonitoring(void) { return false; }
    void failsafeStartMonitoring(void) {}
    void failsafeUpdateState(void) {}
    bool failsafeIsActive(void) { return false; }
    bool rxAreFlightChannelsValid(void) { return false; }
    bool failsafeIsReceivingRxData(void) { return false; }
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
    void baroSetGroundLevel(void) {}
    void changePidProfile(uint8_t) {}
    void changeControlRateProfile(uint8_t) {}
    void dashboardEnablePageCycling(void) {}
    void dashboardDisablePageCycling(void) {}
    bool imuQuaternionHeadfreeOffsetSet(void) { return true; }
    void rescheduleTask(taskId_e, timeDelta_t) {}
    bool usbCableIsInserted(void) { return false; }
    bool usbVcpIsConnected(void) { return false; }
    void pidSetAntiGravityState(bool newState) { UNUSED(newState); }
    void osdSuppressStats(bool) {}
    void pidSetItermReset(bool) {}
    void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
    bool isFixedWing(void) { return false; }
    void compassStartCalibration(void) {}
    bool compassIsCalibrationComplete(void) { return true; }
    bool isUpright(void) { return true; }
    void blackboxLogEvent(FlightLogEvent, union flightLogEventData_u *) {};
    void gyroFiltering(timeUs_t) {};
    timeDelta_t rxGetFrameDelta() { return 0; }
    void updateRcRefreshRate(timeUs_t) {};
    uint16_t getAverageSystemLoadPercent(void) { return 0; }
    bool isMotorProtocolEnabled(void) { return false; }
    void pinioBoxTaskControl(void) {}
    void sbufWriteU8(sbuf_t *, uint8_t) {}
    void sbufWriteU16(sbuf_t *, uint16_t) {}
    void sbufWriteU32(sbuf_t *, uint32_t) {}
    void schedulerSetNextStateTime(timeDelta_t) {}
    bool canUseGPSHeading;
}
