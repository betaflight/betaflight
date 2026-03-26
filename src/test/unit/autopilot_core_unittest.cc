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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

    #include "flight/autopilot_waypoint.h"
    #include "flight/failsafe.h"
    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/position.h"
    #include "flight/servos.h"

    #include "io/beeper.h"
    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/flight_plan.h"
    #include "pg/gps_rescue.h"
    #include "pg/motor.h"
    #include "pg/rx.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "rx/rx.h"

    #include "scheduler/scheduler.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    #include "telemetry/telemetry.h"

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
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(flightPlanConfig_t, flightPlanConfig, PG_FLIGHT_PLAN_CONFIG, 0);

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
    uint32_t GPS_distanceToHomeCm = 0;
    int16_t GPS_directionToHome = 0;
    acc_t acc = {};
    bool mockIsUpright = false;
    uint8_t activePidLoopDenom = 1;

    float getGpsDataIntervalSeconds(void) { return 0.1f; }
    float getGpsDataFrequencyHz(void) { return 10.0f; }

    // Controllable waypoint mock state
    static waypointState_e mockWaypointState = WP_STATE_IDLE;
    static bool mockEmergencyLandingCalled = false;
    static bool mockWaypointResetCalled = false;
    static bool mockWaypointResumeCalled = false;
    static gpsLocation_t mockWaypointTarget = {};
}

uint32_t simulationFeatureFlags = 0;
uint32_t simulationTime = 0;
bool gyroCalibDone = false;
bool simulationHaveRx = true;

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void resetCoreTestState(void)
{
    simulationTime = 10000000;  // Past power-on delay
    simulationFeatureFlags = 0;
    gyroCalibDone = true;
    simulationHaveRx = true;

    mockWaypointState = WP_STATE_IDLE;
    mockEmergencyLandingCalled = false;
    mockWaypointResetCalled = false;
    mockWaypointResumeCalled = false;
    memset(&mockWaypointTarget, 0, sizeof(mockWaypointTarget));

    flightModeFlags = 0;
    armingFlags = 0;
    stateFlags = 0;
    sensorsClear(0xFFFFFFFF);

    memset(&gpsSol, 0, sizeof(gpsSol));

    // Reset PG configs
    flightPlanConfigMutable()->waypointCount = 0;

    // Set ALL channels to center (1500) first, including ROLL/PITCH/YAW
    // This prevents processRcStickPositions from detecting arm/disarm stick combos
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = 1500;
    }
    rcData[THROTTLE] = 1000;
    // Keep BOXARM active (AUX2=1800) to prevent rcDisarmTicks accumulation
    // in processRcStickPositions across test runs
    rcData[AUX2] = 1800;
}

static void setupAutopilotBoxMode(void)
{
    // Configure BOXAUTOPILOT on AUX1 (channel index 4, auxChannelIndex 0)
    modeActivationConditionsMutable(0)->modeId = BOXAUTOPILOT;
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // Configure BOXARM on AUX2 (channel index 5, auxChannelIndex 1)
    // This prevents processRcStickPositions from accumulating rcDisarmTicks
    // (which causes disarm after 4 calls when BOXARM is not configured)
    modeActivationConditionsMutable(1)->modeId = BOXARM;
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    rcControlsInit();
}

static void enableAutopilotBox(void)
{
    rcData[AUX1] = 1800;  // Within the activation range
    updateActivatedModes();
}

static void disableAutopilotBox(void)
{
    rcData[AUX1] = 1000;  // Outside the activation range
    updateActivatedModes();
}

static void setArmedWithGps(void)
{
    armingFlags |= ARMED;
    sensorsSet(SENSOR_GPS);
    stateFlags |= GPS_FIX;
}

static void setupWaypointMission(uint8_t count)
{
    flightPlanConfigMutable()->waypointCount = count;
    for (uint8_t i = 0; i < count; i++) {
        flightPlanConfigMutable()->waypoints[i].latitude = 400000000 + i * 1000000;
        flightPlanConfigMutable()->waypoints[i].longitude = 100000000;
        flightPlanConfigMutable()->waypoints[i].altitude = 50000;
        flightPlanConfigMutable()->waypoints[i].type = WAYPOINT_TYPE_FLYOVER;
    }
}

static void simulateRxLoss(void)
{
    // Set FAILSAFE_MODE to simulate RX loss
    flightModeFlags |= FAILSAFE_MODE;
}

// =========================================================================
// RX Loss Policy Tests
// =========================================================================

TEST(AutopilotCoreUnittest, AutopilotActivatesWithBoxAndGps)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);

    enableAutopilotBox();
    processRxModes(simulationTime);

    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_TRUE(FLIGHT_MODE(ANGLE_MODE));
    EXPECT_TRUE(mockWaypointResumeCalled);
}

TEST(AutopilotCoreUnittest, AutopilotDoesNotActivateWithoutGps)
{
    resetCoreTestState();
    setupAutopilotBoxMode();

    armingFlags |= ARMED;
    // No GPS fix
    setupWaypointMission(3);

    enableAutopilotBox();
    processRxModes(simulationTime);

    EXPECT_FALSE(FLIGHT_MODE(AUTOPILOT_MODE));
}

TEST(AutopilotCoreUnittest, AutopilotDoesNotActivateWithoutWaypoints)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    // No waypoints (count = 0)

    enableAutopilotBox();
    processRxModes(simulationTime);

    EXPECT_FALSE(FLIGHT_MODE(AUTOPILOT_MODE));
}

TEST(AutopilotCoreUnittest, AutopilotDeactivatesWhenBoxDisabled)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);

    // Activate
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Deactivate box
    disableAutopilotBox();
    processRxModes(simulationTime);

    EXPECT_FALSE(FLIGHT_MODE(AUTOPILOT_MODE));
}

TEST(AutopilotCoreUnittest, RxLossPolicyDisable)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_DISABLE;

    // Activate autopilot normally
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Simulate RX loss
    simulateRxLoss();
    disableAutopilotBox();  // Box is no longer readable
    processRxModes(simulationTime);

    // Policy = DISABLE → autopilot should be disabled
    EXPECT_FALSE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_FALSE(mockEmergencyLandingCalled);
}

TEST(AutopilotCoreUnittest, RxLossPolicyContinue)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_CONTINUE;

    // Activate autopilot normally
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Simulate RX loss
    simulateRxLoss();
    disableAutopilotBox();
    processRxModes(simulationTime);

    // Policy = CONTINUE → autopilot should remain active
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_FALSE(mockEmergencyLandingCalled);
}

TEST(AutopilotCoreUnittest, RxLossPolicyLand)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_LAND;

    // Activate autopilot normally (waypointReset sets state to APPROACHING)
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Reset the flag since we only care about emergency landing from RX loss
    mockEmergencyLandingCalled = false;

    // Simulate RX loss
    simulateRxLoss();
    disableAutopilotBox();
    processRxModes(simulationTime);

    // Policy = LAND → autopilot stays active + emergency landing triggered
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_TRUE(mockEmergencyLandingCalled);
}

TEST(AutopilotCoreUnittest, RxLossLandDoesNotRetriggerWhenAlreadyLanding)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_LAND;

    // Activate autopilot first (waypointReset sets state to APPROACHING)
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Now simulate that landing is already in progress
    mockWaypointState = WP_STATE_LANDING;
    mockEmergencyLandingCalled = false;

    // Simulate RX loss while already landing
    simulateRxLoss();
    disableAutopilotBox();
    processRxModes(simulationTime);

    // Should NOT call emergency landing again (already landing)
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_FALSE(mockEmergencyLandingCalled);
}

TEST(AutopilotCoreUnittest, RxLossLandDoesNotRetriggerWhenComplete)
{
    resetCoreTestState();
    setupAutopilotBoxMode();
    setArmedWithGps();
    setupWaypointMission(3);
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_LAND;

    // Activate autopilot first (waypointReset sets state to APPROACHING)
    enableAutopilotBox();
    processRxModes(simulationTime);
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));

    // Now simulate that mission is already complete
    mockWaypointState = WP_STATE_COMPLETE;
    mockEmergencyLandingCalled = false;

    // Simulate RX loss when complete
    simulateRxLoss();
    disableAutopilotBox();
    processRxModes(simulationTime);

    // Should NOT call emergency landing (already complete)
    EXPECT_TRUE(FLIGHT_MODE(AUTOPILOT_MODE));
    EXPECT_FALSE(mockEmergencyLandingCalled);
}

// =========================================================================
// STUBS
// =========================================================================

extern "C" {
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
    void gyroUpdate(void) {}
    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 0; }
    void updateRSSI(timeUs_t) {}
    bool failsafeIsMonitoring(void) { return false; }
    void failsafeStartMonitoring(void) {}
    void failsafeUpdateState(void) {}
    bool failsafeIsActive(void) { return FLIGHT_MODE(FAILSAFE_MODE); }
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
    bool accHasBeenCalibrated(void) { return true; }
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
    bool crashRecoveryModeActive(void) { return false; }
    int32_t getEstimatedAltitudeCm(void) { return 0; }
    bool gpsIsHealthy(void) { return false; }
    float getCosTiltAngle(void) { return 0.0f; }
    void pidSetItermReset(bool) {}
    void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
    bool isFixedWing(void) { return false; }
    void compassStartCalibration(void) {}
    bool compassIsCalibrationComplete(void) { return true; }
    bool isUpright(void) { return mockIsUpright; }
    void blackboxLogEvent(FlightLogEvent, union flightLogEventData_u *) {};
    void gyroFiltering(timeUs_t) {};
    timeDelta_t rxGetFrameDelta() { return 0; }
    void updateRcRefreshRate(timeUs_t) {};
    uint16_t getAverageSystemLoadPercent(void) { return 0; }
    bool isMotorProtocolEnabled(void) { return true; }
    void pinioBoxTaskControl(void) {}
    void schedulerSetNextStateTime(timeDelta_t) {}

    float getAltitudeCm(void) { return 0.0f; }
    float getAltitudeDerivative(void) { return 0.0f; }

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

    void GPS_distance2d(const gpsLocation_t*, const gpsLocation_t*, vector2_t*) { }

    void navOriginUpdate(const gpsLocation_t*) { }
    void navOriginLLHtoNED(const gpsLocation_t*, vector3_t*) { }

    bool canUseGPSHeading;
    bool compassIsHealthy;

    bool gpsHasNewData(uint16_t* gpsStamp) {
        UNUSED(*gpsStamp);
        return true;
    }

    // Waypoint stubs (controllable via mock state)
    waypointState_e waypointGetState(void) {
        return mockWaypointState;
    }

    void waypointSetEmergencyLanding(void) {
        mockEmergencyLandingCalled = true;
        mockWaypointState = WP_STATE_LANDING;
    }

    void waypointReset(void) {
        mockWaypointResetCalled = true;
        mockWaypointState = WP_STATE_APPROACHING;
    }

    void waypointResume(void) {
        mockWaypointResumeCalled = true;
        mockWaypointState = WP_STATE_APPROACHING;
    }

    const gpsLocation_t* waypointGetTarget(void) {
        return &mockWaypointTarget;
    }

    bool waypointIsSystemValid(void) {
        return mockWaypointState != WP_STATE_IDLE;
    }

    // Autopilot controller stubs
    void resetPositionControl(const gpsLocation_t*, uint32_t) {}
    void resetAltitudeControl(void) {}

    // Altitude hold stubs
    bool isAltitudeAvailable(void) { return false; }
    bool navOriginIsValid(void) { return true; }

    // Position hold stubs
    void posHoldInit(void) {}
    void posHoldNewGpsData(void) {}
}
