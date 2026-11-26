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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include <deque>
#include <string>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "telemetry/mavlink_lib.h"

    #include "build/debug.h"
    #include "build/version.h"
    #include "common/time.h"
    #include "config/feature.h"
    #include "drivers/system.h"
    #include "fc/core.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/failsafe.h"
    #include "flight/flight_plan_nav.h"
    #include "flight/imu.h"
    #include "flight/mixer.h"
    #include "flight/position.h"
    #include "io/gps.h"
    #include "io/serial.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "sensors/gyro.h"
    #include "sensors/sensors.h"
    #include "rx/rx.h"
    #include "sensors/battery.h"
    #include "telemetry/telemetry.h"
    #include "telemetry/mavlink.h"

    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    uint8_t armingFlags;
    uint8_t stateFlags;
    uint16_t flightModeFlags;
    uint16_t enableFlightMode(flightModeFlags_e mask)
    {
        flightModeFlags |= mask;
        return flightModeFlags;
    }
    uint16_t disableFlightMode(flightModeFlags_e mask)
    {
        flightModeFlags &= ~mask;
        return flightModeFlags;
    }

    const uint32_t baudRates[BAUD_COUNT] = {
        0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000
    };

    static uint32_t fakeMillis;
    static uint32_t fakeMicros;
    uint32_t millis(void) { return fakeMillis; }
    uint32_t micros(void) { return fakeMicros; }

    static serialPort_t testPort;
    static std::vector<uint8_t> *txBytes;
    static std::deque<uint8_t> *rxBytes;
    serialPort_t *telemetrySharedPort;
    serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr,
        void *, uint32_t, portMode_e, portOptions_e)
    {
        return &testPort;
    }
    void closeSerialPort(serialPort_t *) {}
    void serialWrite(serialPort_t *, uint8_t ch) { txBytes->push_back(ch); }
    static uint32_t txFree;
    uint32_t serialTxBytesFree(const serialPort_t *) { return txFree; }
    uint32_t serialRxBytesWaiting(const serialPort_t *) { return rxBytes->size(); }
    uint8_t serialRead(serialPort_t *)
    {
        const uint8_t c = rxBytes->front();
        rxBytes->pop_front();
        return c;
    }
    static int waitForTxCalls;
    void waitForSerialPortToFinishTransmitting(serialPort_t *) { waitForTxCalls++; }
    portSharing_e determinePortSharing(serialPortIdentifier_e, serialPortFunction_e) { return PORTSHARING_NOT_SHARED; }

    serialPortIdentifier_e telemetryProviderPort(uint8_t) { return SERIAL_PORT_USART1; }
    uint8_t telemetryProviderBaud(uint8_t, serialPortIdentifier_e) { return BAUD_AUTO; }
    bool telemetryCheckRxPortShared(serialPortIdentifier_e, const SerialRXType) { return false; }
    bool telemetryDetermineEnabledState(portSharing_e) { return true; }

    rxRuntimeState_t rxRuntimeState;
    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    uint16_t getRssi(void) { return 0; }

    static int disarmCalls;
    static flightLogDisarmReason_e lastDisarmReason;
    void disarm(flightLogDisarmReason_e reason)
    {
        disarmCalls++;
        lastDisarmReason = reason;
        DISABLE_ARMING_FLAG(ARMED);
    }
    static int tryArmCalls;
    static bool tryArmSucceeds;
    void tryArm(void)
    {
        tryArmCalls++;
        if (tryArmSucceeds) {
            ENABLE_ARMING_FLAG(ARMED);
        }
    }
    static armingDisableFlags_e armingDisableFlags;
    armingDisableFlags_e getArmingDisableFlags(void) { return armingDisableFlags; }
    bool isArmingDisabled(void) { return armingDisableFlags != 0; }
    const char *getArmingDisableFlagName(armingDisableFlags_e flag)
    {
        switch (flag) {
        case ARMING_DISABLED_RX_FAILSAFE: return "RXLOSS";
        case ARMING_DISABLED_THROTTLE: return "THROTTLE";
        default: return "OTHER";
        }
    }

    static uint32_t overrideMask;
    static int overrideClears;
    void rcModeSetExternalOverride(boxId_e boxId, bool enabled)
    {
        if (enabled) {
            overrideMask |= 1u << boxId;
        } else {
            overrideMask &= ~(1u << boxId);
        }
    }
    void rcModeClearExternalOverrides(void)
    {
        overrideMask = 0;
        overrideClears++;
    }

    static bool failsafeActive;
    bool failsafeIsActive(void) { return failsafeActive; }
    static failsafePhase_e fsPhase;
    failsafePhase_e failsafePhase(void) { return fsPhase; }
    static bool throttleRaised;
    bool wasThrottleRaised(void) { return throttleRaised; }

    attitudeEulerAngles_t attitude;
    int32_t getEstimatedAltitudeCm(void) { return 0; }
    static float altitudeDerivative;
    float getAltitudeDerivative(void) { return altitudeDerivative; }

    gyro_t gyro;
    float motor[MAX_SUPPORTED_MOTORS];
    static float motorStopped;
    float getMotorOutputDisarmed(void) { return motorStopped; }
    static uint8_t motorCount;
    uint8_t getMotorCount(void) { return motorCount; }
    uint16_t motorConvertToExternal(float motorValue) { return lrintf(motorValue); }

    static uint32_t sensorMask;
    bool sensors(uint32_t mask) { return (sensorMask & mask) != 0; }

    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;
    static int resetHomeCalls;
    void GPS_reset_home_position(void)
    {
        resetHomeCalls++;
        ENABLE_STATE(GPS_FIX_HOME);
    }

    static batteryState_e batteryState;
    static bool batteryConfigured;
    batteryState_e getBatteryState(void) { return batteryState; }
    bool isBatteryVoltageConfigured(void) { return batteryConfigured; }

    static uint32_t featureMask;
    bool featureIsEnabled(const uint32_t mask) { return (featureMask & mask) != 0; }

    static bool gpsHealthy;
    bool gpsIsHealthy(void) { return gpsHealthy; }
    static bool rxReceiving;
    bool isRxReceivingSignal(void) { return rxReceiving; }
    static uint16_t systemLoadPercent;
    uint16_t getAverageSystemLoadPercent(void) { return systemLoadPercent; }
    uint16_t getBatteryVoltage(void) { return 0; }
    bool isAmperageConfigured(void) { return false; }
    int32_t getAmperage(void) { return 0; }
    uint8_t calculateBatteryPercentageRemaining(void) { return 0; }
    int32_t getMAhDrawn(void) { return 0; }
    uint8_t getBatteryCellCount(void) { return 0; }
    uint16_t getBatteryAverageCellVoltage(void) { return 0; }

    static int motorShutdownCalls;
    void motorShutdown(void) { motorShutdownCalls++; }
    static int systemResetCalls;
    void systemReset(void) { systemResetCalls++; }
    static int bootloaderResetCalls;
    void systemResetToBootloader(bootloaderRequestType_e) { bootloaderResetCalls++; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define GCS_SYS 255
#define GCS_COMP MAV_COMP_ID_MISSIONPLANNER
#define FC_SYS 1
#define FC_COMP MAV_COMP_ID_AUTOPILOT1

enum {
    MODE_ACRO = 0,
    MODE_ANGLE,
    MODE_HORIZON,
    MODE_ALT_HOLD,
    MODE_POS_HOLD,
    MODE_AUTOPILOT,
    MODE_RTL,
    MODE_FAILSAFE,
    MODE_COUNT,
};

class MavlinkTelemetryTest : public ::testing::Test {
protected:
    std::vector<uint8_t> tx;
    std::deque<uint8_t> rx;
    std::vector<mavlink_message_t> sent;

    void SetUp() override
    {
        txBytes = &tx;
        rxBytes = &rx;

        fakeMillis += 10000;
        fakeMicros = fakeMillis * 1000;
        armingFlags = 0;
        stateFlags = 0;
        flightModeFlags = 0;
        armingDisableFlags = (armingDisableFlags_e)0;
        disarmCalls = 0;
        tryArmCalls = 0;
        tryArmSucceeds = false;
        overrideMask = 0;
        overrideClears = 0;
        failsafeActive = false;
        fsPhase = FAILSAFE_IDLE;
        throttleRaised = false;
        sensorMask = 0;
        batteryState = BATTERY_NOT_PRESENT;
        batteryConfigured = false;
        featureMask = 0;
        gpsHealthy = false;
        rxReceiving = true;
        systemLoadPercent = 0;
        resetHomeCalls = 0;
        motorShutdownCalls = 0;
        systemResetCalls = 0;
        bootloaderResetCalls = 0;
        waitForTxCalls = 0;
        memset(&attitude, 0, sizeof(attitude));
        memset(&gyro, 0, sizeof(gyro));
        altitudeDerivative = 0;
        memset(motor, 0, sizeof(motor));
        motorStopped = 0;
        motorCount = 4;
        txFree = 1024;
        memset(&gpsSol, 0, sizeof(gpsSol));
        memset(&GPS_home_llh, 0, sizeof(GPS_home_llh));
        mixerConfigMutable()->mixerMode = MIXER_QUADX;

        telemetryConfig_t *cfg = telemetryConfigMutable();
        cfg->mavlink_extended_status_rate = 0;
        cfg->mavlink_rc_channels_rate = 0;
        cfg->mavlink_position_rate = 0;
        cfg->mavlink_extra1_rate = 0;
        cfg->mavlink_extra2_rate = 0;
        cfg->mavlink_extra3_rate = 0;

        mavlink_reset_channel_status(MAVLINK_COMM_0);

        freeMAVLinkTelemetryPort();
        initMAVLinkTelemetry();
        checkMAVLinkTelemetryState();
        tx.clear();
    }

    void enableStreams(uint8_t extra2Rate)
    {
        telemetryConfigMutable()->mavlink_extra2_rate = extra2Rate;
        freeMAVLinkTelemetryPort();
        checkMAVLinkTelemetryState();
        tx.clear();
    }

    void advanceMs(uint32_t ms)
    {
        fakeMillis += ms;
        fakeMicros += ms * 1000;
    }

    void feed(const mavlink_message_t *msg)
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        const uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
        rx.insert(rx.end(), buf, buf + len);
        while (!rx.empty()) {
            handleMAVLinkTelemetry();
        }
        collect();
    }

    void tick(uint32_t ms)
    {
        advanceMs(ms);
        handleMAVLinkTelemetry();
        collect();
    }

    void collect()
    {
        mavlink_message_t msg;
        mavlink_status_t status;
        for (uint8_t c : tx) {
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status) == MAVLINK_FRAMING_OK) {
                sent.push_back(msg);
            }
        }
        tx.clear();
    }

    const mavlink_message_t *lastOf(uint32_t msgid)
    {
        for (auto it = sent.rbegin(); it != sent.rend(); ++it) {
            if (it->msgid == msgid) {
                return &(*it);
            }
        }
        return nullptr;
    }

    int countOf(uint32_t msgid)
    {
        int n = 0;
        for (const auto &m : sent) {
            n += (m.msgid == msgid);
        }
        return n;
    }

    void commandLong(uint16_t command, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0,
        float p5 = 0, float p6 = 0, float p7 = 0, uint8_t targetSys = FC_SYS, uint8_t targetComp = FC_COMP)
    {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg,
            targetSys, targetComp, command, 0, p1, p2, p3, p4, p5, p6, p7);
        feed(&msg);
    }

    void commandInt(uint16_t command, float p1, int32_t x, int32_t y, float z, float p2 = 0)
    {
        mavlink_message_t msg;
        mavlink_msg_command_int_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg,
            FC_SYS, FC_COMP, MAV_FRAME_GLOBAL_INT, command, 0, 0, p1, p2, 0, 0, x, y, z);
        feed(&msg);
    }

    uint8_t intAckResultFor(uint16_t command, float p1 = 0, float p2 = 0)
    {
        sent.clear();
        commandInt(command, p1, 0, 0, 0, p2);
        const mavlink_command_ack_t ack = lastAck();
        EXPECT_EQ(ack.command, command);
        EXPECT_EQ(ack.target_system, GCS_SYS);
        EXPECT_EQ(ack.target_component, GCS_COMP);
        return ack.result;
    }

    mavlink_command_ack_t lastAck()
    {
        mavlink_command_ack_t ack;
        memset(&ack, 0, sizeof(ack));
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_COMMAND_ACK);
        EXPECT_NE(m, nullptr);
        if (m) {
            mavlink_msg_command_ack_decode(m, &ack);
        }
        return ack;
    }

    uint8_t ackResultFor(uint16_t command, float p1 = 0, float p2 = 0)
    {
        sent.clear();
        commandLong(command, p1, p2);
        const mavlink_command_ack_t ack = lastAck();
        EXPECT_EQ(ack.command, command);
        EXPECT_EQ(ack.target_system, GCS_SYS);
        EXPECT_EQ(ack.target_component, GCS_COMP);
        return ack.result;
    }

    mavlink_heartbeat_t heartbeat()
    {
        sent.clear();
        tick(1100);
        mavlink_heartbeat_t hb;
        memset(&hb, 0, sizeof(hb));
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_HEARTBEAT);
        EXPECT_NE(m, nullptr);
        if (m) {
            EXPECT_EQ(m->sysid, FC_SYS);
            EXPECT_EQ(m->compid, FC_COMP);
            mavlink_msg_heartbeat_decode(m, &hb);
        }
        return hb;
    }

    mavlink_sys_status_t sysStatus()
    {
        telemetryConfigMutable()->mavlink_extended_status_rate = 1;
        freeMAVLinkTelemetryPort();
        checkMAVLinkTelemetryState();
        sent.clear();
        tick(1100);
        mavlink_sys_status_t st;
        memset(&st, 0, sizeof(st));
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_SYS_STATUS);
        EXPECT_NE(m, nullptr);
        if (m) {
            mavlink_msg_sys_status_decode(m, &st);
        }
        return st;
    }

    void setMode(uint32_t customMode)
    {
        sent.clear();
        commandLong(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, customMode);
    }
};

TEST_F(MavlinkTelemetryTest, HeartbeatDisarmedAcro)
{
    enableStreams(1);
    const mavlink_heartbeat_t hb = heartbeat();
    EXPECT_EQ(hb.type, MAV_TYPE_QUADROTOR);
    EXPECT_EQ(hb.autopilot, MAV_AUTOPILOT_BETAFLIGHT);
    EXPECT_EQ(hb.base_mode, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_ACRO);
    EXPECT_EQ(hb.system_status, MAV_STATE_STANDBY);
}

TEST_F(MavlinkTelemetryTest, HeartbeatArmedAngle)
{
    enableStreams(1);
    ENABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    const mavlink_heartbeat_t hb = heartbeat();
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_STABILIZE_ENABLED);
    EXPECT_TRUE(hb.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED);
    EXPECT_FALSE(hb.base_mode & (MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED));
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_ANGLE);
    EXPECT_EQ(hb.system_status, MAV_STATE_ACTIVE);
}

TEST_F(MavlinkTelemetryTest, HeartbeatArmedFailsafeIsCritical)
{
    enableStreams(1);
    ENABLE_ARMING_FLAG(ARMED);
    failsafeActive = true;
    const mavlink_heartbeat_t hb = heartbeat();
    EXPECT_EQ(hb.custom_mode, (uint32_t)MODE_FAILSAFE);
    EXPECT_EQ(hb.system_status, MAV_STATE_CRITICAL);
}

TEST_F(MavlinkTelemetryTest, HeartbeatCustomModePriority)
{
    enableStreams(1);
    ENABLE_ARMING_FLAG(ARMED);

    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    ENABLE_FLIGHT_MODE(HORIZON_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_HORIZON);

    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_ALT_HOLD);

    ENABLE_FLIGHT_MODE(POS_HOLD_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_POS_HOLD);

    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_AUTOPILOT);

    ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_RTL);

    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    EXPECT_EQ(heartbeat().custom_mode, (uint32_t)MODE_FAILSAFE);
}

TEST_F(MavlinkTelemetryTest, HeartbeatFlagsAutonomousModes)
{
    const uint8_t autonomous = MAV_MODE_FLAG_GUIDED_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED;
    ENABLE_ARMING_FLAG(ARMED);

    ENABLE_FLIGHT_MODE(POS_HOLD_MODE);
    EXPECT_EQ(heartbeat().base_mode & autonomous, 0);

    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    EXPECT_EQ(heartbeat().base_mode & autonomous, autonomous);

    DISABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    EXPECT_EQ(heartbeat().base_mode & autonomous, autonomous);
}

TEST_F(MavlinkTelemetryTest, SysStatusBareAcroQuadIsHealthy)
{
    const uint32_t expected = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS | MAV_SYS_STATUS_SENSOR_RC_RECEIVER | MAV_SYS_STATUS_PREARM_CHECK;
    const mavlink_sys_status_t st = sysStatus();
    EXPECT_EQ(st.onboard_control_sensors_present, expected);
    EXPECT_EQ(st.onboard_control_sensors_enabled, expected & ~MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
    EXPECT_EQ(st.onboard_control_sensors_health, expected);
}

TEST_F(MavlinkTelemetryTest, SysStatusSensorsArePresent)
{
    sensorMask = SENSOR_ACC | SENSOR_MAG | SENSOR_BARO;
    featureMask = FEATURE_GPS;
    batteryConfigured = true;
    batteryState = BATTERY_OK;
    gpsHealthy = true;
    const mavlink_sys_status_t st = sysStatus();
    const uint32_t sensorBits = MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG |
        MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_BATTERY;
    const uint32_t controllers = MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_YAW_POSITION |
        MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    EXPECT_EQ(st.onboard_control_sensors_present & (sensorBits | controllers), sensorBits | controllers);
    EXPECT_EQ(st.onboard_control_sensors_enabled & (sensorBits | controllers), sensorBits);
    EXPECT_EQ(st.onboard_control_sensors_health & (sensorBits | controllers), sensorBits | controllers);
}

TEST_F(MavlinkTelemetryTest, SysStatusControllersFollowFlightMode)
{
    sensorMask = SENSOR_ACC | SENSOR_MAG | SENSOR_BARO;
    featureMask = FEATURE_GPS;
    const uint32_t att = MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION;
    const uint32_t yaw = MAV_SYS_STATUS_SENSOR_YAW_POSITION;
    const uint32_t z = MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    const uint32_t xy = MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    const uint32_t mask = att | yaw | z | xy | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);

    DISABLE_ARMING_FLAG(ARMED);
    ENABLE_FLIGHT_MODE(ANGLE_MODE);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att);

    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att | z);

    ENABLE_FLIGHT_MODE(POS_HOLD_MODE);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att | z | xy);

    flightModeFlags = 0;
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att | yaw | z | xy);

    flightModeFlags = 0;
    ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att | yaw | z | xy);

    failsafeActive = true;
    EXPECT_EQ(sysStatus().onboard_control_sensors_enabled & mask, att | yaw | z | xy);
}

TEST_F(MavlinkTelemetryTest, SysStatusGpsTimeoutIsPresentButUnhealthy)
{
    const uint32_t gpsBits = MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
    EXPECT_EQ(sysStatus().onboard_control_sensors_present & gpsBits, 0u);

    featureMask = FEATURE_GPS;
    gpsHealthy = false;
    const mavlink_sys_status_t st = sysStatus();
    EXPECT_EQ(st.onboard_control_sensors_present & gpsBits, gpsBits);
    EXPECT_FALSE(st.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_GPS);
}

TEST_F(MavlinkTelemetryTest, SysStatusReportsUnhealthySubsystems)
{
    sensorMask = SENSOR_ACC;
    featureMask = FEATURE_GPS;
    batteryConfigured = true;
    batteryState = BATTERY_CRITICAL;
    gpsHealthy = false;
    rxReceiving = false;
    armingDisableFlags = (armingDisableFlags_e)(ARMING_DISABLED_NO_GYRO | ARMING_DISABLED_ACC_CALIBRATION | ARMING_DISABLED_MOTOR_PROTOCOL);
    const uint32_t bad = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_GPS |
        MAV_SYS_STATUS_SENSOR_RC_RECEIVER | MAV_SYS_STATUS_SENSOR_BATTERY | MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS |
        MAV_SYS_STATUS_PREARM_CHECK;
    const mavlink_sys_status_t st = sysStatus();
    EXPECT_EQ(st.onboard_control_sensors_present & bad, bad);
    EXPECT_EQ(st.onboard_control_sensors_health & bad, 0u);
    EXPECT_TRUE(st.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL);
}

TEST_F(MavlinkTelemetryTest, SysStatusPrearmCheck)
{
    armingDisableFlags = ARMING_DISABLED_THROTTLE;
    EXPECT_FALSE(sysStatus().onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK);

    armingDisableFlags = (armingDisableFlags_e)0;
    EXPECT_TRUE(sysStatus().onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK);

    armingDisableFlags = ARMING_DISABLED_THROTTLE;
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_TRUE(sysStatus().onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK);
}

TEST_F(MavlinkTelemetryTest, SysStatusReportsLoad)
{
    systemLoadPercent = 42;
    EXPECT_EQ(sysStatus().load, 420);
}

TEST_F(MavlinkTelemetryTest, HeartbeatTypeFollowsMixer)
{
    enableStreams(1);
    mixerConfigMutable()->mixerMode = MIXER_HEX6X;
    EXPECT_EQ(heartbeat().type, MAV_TYPE_HEXAROTOR);
    mixerConfigMutable()->mixerMode = MIXER_FLYING_WING;
    EXPECT_EQ(heartbeat().type, MAV_TYPE_FIXED_WING);
}

TEST_F(MavlinkTelemetryTest, HeartbeatAloneAtOneHzWhenStreamRatesAreZero)
{
    tick(1100);
    sent.clear();
    for (int i = 0; i < 10; i++) {
        tick(1000);
    }
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_HEARTBEAT), 10);
    EXPECT_EQ(sent.size(), 10u);
}

TEST_F(MavlinkTelemetryTest, HeartbeatIgnoresExtra2Rate)
{
    enableStreams(10);
    tick(1100);
    sent.clear();
    for (int i = 0; i < 10; i++) {
        tick(100);
    }
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_HEARTBEAT), 1);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 10);
}

TEST_F(MavlinkTelemetryTest, HeartbeatIntervalIsFixed)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, -1), MAV_RESULT_DENIED);
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, 100000), MAV_RESULT_DENIED);
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT, 0), MAV_RESULT_ACCEPTED);

    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_HEARTBEAT), MAV_RESULT_ACCEPTED);
    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    EXPECT_EQ(mi.interval_us, 1000000);
}

TEST_F(MavlinkTelemetryTest, UnknownCommandIsUnsupported)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_DIGICAM_CONTROL), MAV_RESULT_UNSUPPORTED);
}

TEST_F(MavlinkTelemetryTest, CommandForAnotherSystemIsIgnored)
{
    commandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, FC_SYS + 1, FC_COMP);
    commandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, FC_SYS, MAV_COMP_ID_CAMERA);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_COMMAND_ACK), 0);
    EXPECT_EQ(disarmCalls, 0);
}

TEST_F(MavlinkTelemetryTest, BroadcastCommandIsAccepted)
{
    commandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
}

TEST_F(MavlinkTelemetryTest, DisarmIsAlwaysAccepted)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 0), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(disarmCalls, 1);
    EXPECT_EQ(lastDisarmReason, DISARM_REASON_SERIAL_COMMAND);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
}

TEST_F(MavlinkTelemetryTest, ForceDisarmIsAccepted)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(disarmCalls, 1);
}

TEST_F(MavlinkTelemetryTest, ArmDisarmRejectsInvalidParam)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 2), MAV_RESULT_DENIED);
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, NAN), MAV_RESULT_DENIED);
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, -1), MAV_RESULT_DENIED);
    EXPECT_EQ(disarmCalls, 0);
    EXPECT_EQ(tryArmCalls, 0);
}

#if ENABLE_REMOTE_ARM
TEST_F(MavlinkTelemetryTest, ArmRoutesThroughTryArm)
{
    tryArmSucceeds = true;
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(tryArmCalls, 1);
    EXPECT_TRUE(ARMING_FLAG(ARMED));
}

TEST_F(MavlinkTelemetryTest, ArmRefusedWhileArmingDisabled)
{
    tryArmSucceeds = true;
    armingDisableFlags = ARMING_DISABLED_RX_FAILSAFE;
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 1), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(tryArmCalls, 0);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
}

TEST_F(MavlinkTelemetryTest, ArmRejectedWhenTryArmFails)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 1), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(tryArmCalls, 1);
}

TEST_F(MavlinkTelemetryTest, ArmWhileArmedIsAccepted)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(tryArmCalls, 0);
}
#else
TEST_F(MavlinkTelemetryTest, ArmUnsupportedWithoutRemoteArm)
{
    tryArmSucceeds = true;
    EXPECT_EQ(ackResultFor(MAV_CMD_COMPONENT_ARM_DISARM, 1), MAV_RESULT_UNSUPPORTED);
    EXPECT_EQ(tryArmCalls, 0);
    EXPECT_FALSE(ARMING_FLAG(ARMED));
}
#endif

TEST_F(MavlinkTelemetryTest, SetModeNeedsCustomModeFlag)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_STABILIZE_ENABLED, MODE_ANGLE), MAV_RESULT_DENIED);
    EXPECT_EQ(overrideMask, 0u);
}

TEST_F(MavlinkTelemetryTest, SetModeForcesOneBox)
{
    const struct {
        uint32_t mode;
        boxId_e box;
    } cases[] = {
        { MODE_ANGLE, BOXANGLE },
        { MODE_HORIZON, BOXHORIZON },
        { MODE_ALT_HOLD, BOXALTHOLD },
        { MODE_POS_HOLD, BOXPOSHOLD },
        { MODE_AUTOPILOT, BOXAUTOPILOT },
        { MODE_RTL, BOXGPSRESCUE },
    };
    for (const auto &c : cases) {
        setMode(c.mode);
        EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED) << "mode " << c.mode;
        EXPECT_EQ(overrideMask, 1u << c.box) << "mode " << c.mode;
    }
}

TEST_F(MavlinkTelemetryTest, SetModeAcroClearsOverrides)
{
    setMode(MODE_POS_HOLD);
    setMode(MODE_ACRO);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(overrideMask, 0u);
}

TEST_F(MavlinkTelemetryTest, SetModeRejectsFailsafeAndUnknownWithoutTouchingOverride)
{
    setMode(MODE_RTL);
    const uint32_t before = overrideMask;

    setMode(MODE_FAILSAFE);
    EXPECT_EQ(lastAck().result, MAV_RESULT_DENIED);
    EXPECT_EQ(overrideMask, before);

    setMode(MODE_COUNT);
    EXPECT_EQ(lastAck().result, MAV_RESULT_DENIED);
    EXPECT_EQ(overrideMask, before);
}

TEST_F(MavlinkTelemetryTest, ReturnToLaunchNeedsArmed)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_NAV_RETURN_TO_LAUNCH), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(overrideMask, 0u);

    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_NAV_RETURN_TO_LAUNCH), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(overrideMask, 1u << BOXGPSRESCUE);
}

TEST_F(MavlinkTelemetryTest, SetHomeCurrentNeedsFix)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_SET_HOME, 1), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(resetHomeCalls, 0);

    ENABLE_STATE(GPS_FIX);
    EXPECT_EQ(ackResultFor(MAV_CMD_DO_SET_HOME, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(resetHomeCalls, 1);
}

TEST_F(MavlinkTelemetryTest, SetHomeFromCommandLongCoordinates)
{
    sent.clear();
    commandLong(MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, -33.5f, 151.25f, 120.5f);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    EXPECT_NEAR(GPS_home_llh.lat, -335000000, 100);
    EXPECT_NEAR(GPS_home_llh.lon, 1512500000, 100);
    EXPECT_EQ(GPS_home_llh.altCm, 12050);
    EXPECT_TRUE(STATE(GPS_FIX_HOME));
}

TEST_F(MavlinkTelemetryTest, SetHomeFromCommandIntCoordinates)
{
    sent.clear();
    commandInt(MAV_CMD_DO_SET_HOME, 0, -335000001, 1512500001, 42.25f);
    EXPECT_EQ(lastAck().result, MAV_RESULT_ACCEPTED);
    EXPECT_EQ(GPS_home_llh.lat, -335000001);
    EXPECT_EQ(GPS_home_llh.lon, 1512500001);
    EXPECT_EQ(GPS_home_llh.altCm, 4225);
    EXPECT_TRUE(STATE(GPS_FIX_HOME));
}

TEST_F(MavlinkTelemetryTest, SetHomeRejectsMissingCoordinates)
{
    sent.clear();
    commandLong(MAV_CMD_DO_SET_HOME, 0, 0, 0, 0, NAN, NAN, NAN);
    EXPECT_EQ(lastAck().result, MAV_RESULT_DENIED);
    EXPECT_FALSE(STATE(GPS_FIX_HOME));
}

TEST_F(MavlinkTelemetryTest, RebootRefusedWhileArmed)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1), MAV_RESULT_TEMPORARILY_REJECTED);
    EXPECT_EQ(systemResetCalls, 0);
}

TEST_F(MavlinkTelemetryTest, RebootHappensAfterAck)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(waitForTxCalls, 1);
    EXPECT_EQ(motorShutdownCalls, 1);
    EXPECT_EQ(systemResetCalls, 1);
    EXPECT_EQ(bootloaderResetCalls, 0);
}

TEST_F(MavlinkTelemetryTest, RebootToBootloader)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 3), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(bootloaderResetCalls, 1);
    EXPECT_EQ(systemResetCalls, 0);
}

TEST_F(MavlinkTelemetryTest, ShutdownIsUnsupported)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 2), MAV_RESULT_UNSUPPORTED);
    EXPECT_EQ(ackResultFor(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(systemResetCalls, 0);
    EXPECT_EQ(bootloaderResetCalls, 0);
}

TEST_F(MavlinkTelemetryTest, AutopilotCapabilitiesSendsVersion)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_AUTOPILOT_VERSION);
    ASSERT_NE(m, nullptr);
    mavlink_autopilot_version_t v;
    mavlink_msg_autopilot_version_decode(m, &v);
    EXPECT_TRUE(v.capabilities & MAV_PROTOCOL_CAPABILITY_MAVLINK2);
    EXPECT_TRUE(v.capabilities & MAV_PROTOCOL_CAPABILITY_COMMAND_INT);
    EXPECT_FALSE(v.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_INT);
    EXPECT_FALSE(v.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE);
    EXPECT_EQ(v.flight_sw_version >> 24, (uint32_t)(FC_VERSION_YEAR - FC_CALVER_BASE_YEAR));
}

TEST_F(MavlinkTelemetryTest, RequestCommandsAcceptedAsCommandInt)
{
    EXPECT_EQ(intAckResultFor(MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1), MAV_RESULT_ACCEPTED);
    EXPECT_NE(lastOf(MAVLINK_MSG_ID_AUTOPILOT_VERSION), nullptr);

    EXPECT_EQ(intAckResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES, 2), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_AVAILABLE_MODES);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(mavlink_msg_available_modes_get_mode_index(m), 2);

    EXPECT_EQ(intAckResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 100000), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(intAckResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE), MAV_RESULT_ACCEPTED);
    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    EXPECT_EQ(mi.interval_us, 100000);

    EXPECT_EQ(intAckResultFor(MAV_CMD_DO_DIGICAM_CONTROL), MAV_RESULT_UNSUPPORTED);
}

TEST_F(MavlinkTelemetryTest, CommandIntForAnotherSystemIsIgnored)
{
    mavlink_message_t msg;
    mavlink_msg_command_int_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg,
        FC_SYS + 1, FC_COMP, MAV_FRAME_GLOBAL_INT, MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES, 0, 0, 1, 0, 0, 0, 0, 0, 0);
    sent.clear();
    feed(&msg);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_COMMAND_ACK), nullptr);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_AUTOPILOT_VERSION), nullptr);
}

TEST_F(MavlinkTelemetryTest, RequestMessageAutopilotVersion)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AUTOPILOT_VERSION), MAV_RESULT_ACCEPTED);
    EXPECT_NE(lastOf(MAVLINK_MSG_ID_AUTOPILOT_VERSION), nullptr);
}

TEST_F(MavlinkTelemetryTest, RequestMessageUnknownIsUnsupported)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_RAW_IMU), MAV_RESULT_UNSUPPORTED);
}

TEST_F(MavlinkTelemetryTest, AvailableModesDescribeEveryMode)
{
    const struct {
        const char *name;
        uint8_t standardMode;
        uint32_t properties;
    } expected[MODE_COUNT] = {
        { "Acro", MAV_STANDARD_MODE_NON_STANDARD, 0 },
        { "Angle", MAV_STANDARD_MODE_NON_STANDARD, 0 },
        { "Horizon", MAV_STANDARD_MODE_NON_STANDARD, 0 },
        { "Alt Hold", MAV_STANDARD_MODE_ALTITUDE_HOLD, 0 },
        { "Pos Hold", MAV_STANDARD_MODE_POSITION_HOLD, 0 },
        { "Autopilot", MAV_STANDARD_MODE_MISSION, 0 },
        { "RTL", MAV_STANDARD_MODE_SAFE_RECOVERY, 0 },
        { "Failsafe", MAV_STANDARD_MODE_SAFE_RECOVERY, MAV_MODE_PROPERTY_NOT_USER_SELECTABLE },
    };

    for (uint8_t index = 1; index <= MODE_COUNT; index++) {
        EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES, index), MAV_RESULT_ACCEPTED);
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_AVAILABLE_MODES);
        ASSERT_NE(m, nullptr);
        mavlink_available_modes_t am;
        mavlink_msg_available_modes_decode(m, &am);
        const auto &e = expected[index - 1];
        EXPECT_EQ(am.number_modes, MODE_COUNT);
        EXPECT_EQ(am.mode_index, index);
        EXPECT_EQ(am.custom_mode, (uint32_t)(index - 1));
        EXPECT_EQ(am.standard_mode, e.standardMode);
        EXPECT_EQ(am.properties, e.properties);
        EXPECT_EQ(std::string(am.mode_name, strnlen(am.mode_name, sizeof(am.mode_name))), e.name);
    }
}

TEST_F(MavlinkTelemetryTest, AvailableModesRejectsOutOfRangeIndex)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES, MODE_COUNT + 1), MAV_RESULT_DENIED);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_AVAILABLE_MODES), nullptr);
}

TEST_F(MavlinkTelemetryTest, AvailableModesAllAreSentOnePerPass)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES, 0), MAV_RESULT_ACCEPTED);
    EXPECT_LE(countOf(MAVLINK_MSG_ID_AVAILABLE_MODES), 1);

    for (int pass = 0; pass < MODE_COUNT + 2; pass++) {
        const int before = countOf(MAVLINK_MSG_ID_AVAILABLE_MODES);
        tick(20);
        EXPECT_LE(countOf(MAVLINK_MSG_ID_AVAILABLE_MODES) - before, 1);
    }

    std::vector<uint8_t> indexes;
    for (const auto &m : sent) {
        if (m.msgid == MAVLINK_MSG_ID_AVAILABLE_MODES) {
            indexes.push_back(mavlink_msg_available_modes_get_mode_index(&m));
        }
    }
    ASSERT_EQ(indexes.size(), (size_t)MODE_COUNT);
    for (uint8_t i = 0; i < MODE_COUNT; i++) {
        EXPECT_EQ(indexes[i], i + 1);
    }
}

TEST_F(MavlinkTelemetryTest, AvailableModesAllWaitsForTxSpace)
{
    txFree = 0;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES, 0), MAV_RESULT_ACCEPTED);
    tick(20);
    tick(20);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_AVAILABLE_MODES), 0);

    txFree = 1024;
    for (int pass = 0; pass < MODE_COUNT + 2; pass++) {
        tick(20);
    }
    ASSERT_EQ(countOf(MAVLINK_MSG_ID_AVAILABLE_MODES), MODE_COUNT);
    for (const auto &m : sent) {
        if (m.msgid == MAVLINK_MSG_ID_AVAILABLE_MODES) {
            EXPECT_EQ(mavlink_msg_available_modes_get_mode_index(&m), 1);
            break;
        }
    }
}

TEST_F(MavlinkTelemetryTest, AvailableModesMonitorReportsSequence)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(mavlink_msg_available_modes_monitor_get_seq(m), 1);
}

TEST_F(MavlinkTelemetryTest, MessageIntervalReflectsStreamRate)
{
    enableStreams(2);
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL);
    ASSERT_NE(m, nullptr);
    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(m, &mi);
    EXPECT_EQ(mi.message_id, MAVLINK_MSG_ID_VFR_HUD);
    EXPECT_EQ(mi.interval_us, 500000);
}

TEST_F(MavlinkTelemetryTest, MessageIntervalDisabledAndUnknown)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE), MAV_RESULT_ACCEPTED);
    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    EXPECT_EQ(mi.interval_us, -1);

    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_RAW_IMU), MAV_RESULT_ACCEPTED);
    mavlink_msg_message_interval_decode(lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    EXPECT_EQ(mi.interval_us, 0);
}

TEST_F(MavlinkTelemetryTest, SetMessageIntervalChangesRateImmediately)
{
    enableStreams(1);
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_VFR_HUD, 100000), MAV_RESULT_ACCEPTED);

    sent.clear();
    for (int i = 0; i < 10; i++) {
        tick(100);
    }
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 10);
}

TEST_F(MavlinkTelemetryTest, SetMessageIntervalSlowerKeepsPendingSend)
{
    enableStreams(10);
    tick(150);
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_VFR_HUD, 1000000), MAV_RESULT_ACCEPTED);

    sent.clear();
    tick(100);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 1);
    sent.clear();
    for (int i = 0; i < 9; i++) {
        tick(100);
    }
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 0);
    tick(100);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 1);
}

TEST_F(MavlinkTelemetryTest, SetMessageIntervalClampsToMinimum)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 1000), MAV_RESULT_ACCEPTED);
    ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE);
    mavlink_message_interval_t mi;
    mavlink_msg_message_interval_decode(lastOf(MAVLINK_MSG_ID_MESSAGE_INTERVAL), &mi);
    EXPECT_EQ(mi.interval_us, MIN_MAVLINK_TELEMETRY_UPDATE_INTERVAL_MS * 1000);
}

TEST_F(MavlinkTelemetryTest, SetMessageIntervalDisableAndDefault)
{
    enableStreams(1);
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_VFR_HUD, -1), MAV_RESULT_ACCEPTED);
    sent.clear();
    tick(2000);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 0);

    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_VFR_HUD, 0), MAV_RESULT_ACCEPTED);
    sent.clear();
    tick(2000);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_VFR_HUD), 1);
}

TEST_F(MavlinkTelemetryTest, SetMessageIntervalUnknownMessageDenied)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_RAW_IMU, 100000), MAV_RESULT_DENIED);
}

TEST_F(MavlinkTelemetryTest, PingRequestIsAnswered)
{
    mavlink_message_t msg;
    mavlink_msg_ping_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg, 123456, 7, 0, 0);
    feed(&msg);

    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_PING);
    ASSERT_NE(m, nullptr);
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(m, &ping);
    EXPECT_EQ(ping.time_usec, 123456u);
    EXPECT_EQ(ping.seq, 7u);
    EXPECT_EQ(ping.target_system, GCS_SYS);
    EXPECT_EQ(ping.target_component, GCS_COMP);
}

TEST_F(MavlinkTelemetryTest, PingResponseIsNotEchoed)
{
    mavlink_message_t msg;
    mavlink_msg_ping_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg, 123456, 7, FC_SYS, FC_COMP);
    feed(&msg);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_PING), 0);
}

TEST_F(MavlinkTelemetryTest, TimesyncRequestIsAnswered)
{
    mavlink_message_t msg;
    mavlink_msg_timesync_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg, 0, 987654321, 0, 0);
    feed(&msg);

    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_TIMESYNC);
    ASSERT_NE(m, nullptr);
    mavlink_timesync_t ts;
    mavlink_msg_timesync_decode(m, &ts);
    EXPECT_EQ(ts.ts1, 987654321);
    EXPECT_EQ(ts.tc1, (int64_t)fakeMicros * 1000);
    EXPECT_EQ(ts.target_system, GCS_SYS);
}

TEST_F(MavlinkTelemetryTest, TimesyncResponseAndOtherTargetIgnored)
{
    mavlink_message_t msg;
    mavlink_msg_timesync_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg, 5, 987654321, 0, 0);
    feed(&msg);
    mavlink_msg_timesync_pack_chan(GCS_SYS, GCS_COMP, MAVLINK_COMM_1, &msg, 0, 987654321, FC_SYS + 1, 0);
    feed(&msg);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_TIMESYNC), 0);
}

TEST_F(MavlinkTelemetryTest, ArmingDisabledReasonsSentOnChange)
{
    enableStreams(1);
    armingDisableFlags = (armingDisableFlags_e)(ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_THROTTLE);
    sent.clear();
    tick(1000);

    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_STATUSTEXT);
    ASSERT_NE(m, nullptr);
    mavlink_statustext_t st;
    mavlink_msg_statustext_decode(m, &st);
    EXPECT_EQ(st.severity, MAV_SEVERITY_NOTICE);
    const std::string text(st.text, strnlen(st.text, sizeof(st.text)));
    EXPECT_EQ(text, "Arming disabled: RXLOSS THROTTLE");

    sent.clear();
    tick(1000);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_STATUSTEXT), 0);
}

class MavlinkExtendedSysStateTest : public MavlinkTelemetryTest {
protected:
    mavlink_extended_sys_state_t requestState()
    {
        EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_EXTENDED_SYS_STATE), MAV_RESULT_ACCEPTED);
        mavlink_extended_sys_state_t st;
        memset(&st, 0, sizeof(st));
        const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_EXTENDED_SYS_STATE);
        EXPECT_NE(m, nullptr);
        if (m) {
            mavlink_msg_extended_sys_state_decode(m, &st);
        }
        return st;
    }
};

TEST_F(MavlinkExtendedSysStateTest, DisarmedIsOnGround)
{
    const mavlink_extended_sys_state_t st = requestState();
    EXPECT_EQ(st.landed_state, MAV_LANDED_STATE_ON_GROUND);
    EXPECT_EQ(st.vtol_state, MAV_VTOL_STATE_UNDEFINED);
}

TEST_F(MavlinkExtendedSysStateTest, ArmedBeforeThrottleRaisedIsOnGround)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(requestState().landed_state, MAV_LANDED_STATE_ON_GROUND);
}

TEST_F(MavlinkExtendedSysStateTest, ArmedAfterThrottleRaisedIsInAir)
{
    ENABLE_ARMING_FLAG(ARMED);
    throttleRaised = true;
    EXPECT_EQ(requestState().landed_state, MAV_LANDED_STATE_IN_AIR);
}

TEST_F(MavlinkExtendedSysStateTest, FailsafeLandingIsLanding)
{
    ENABLE_ARMING_FLAG(ARMED);
    throttleRaised = true;
    fsPhase = FAILSAFE_LANDING;
    EXPECT_EQ(requestState().landed_state, MAV_LANDED_STATE_LANDING);
}

TEST_F(MavlinkExtendedSysStateTest, StreamsWithExtendedStatus)
{
    telemetryConfigMutable()->mavlink_extended_status_rate = 2;
    freeMAVLinkTelemetryPort();
    checkMAVLinkTelemetryState();
    sent.clear();
    tick(1100);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_EXTENDED_SYS_STATE), 1);
    sent.clear();
    for (int i = 0; i < 10; i++) {
        tick(100);
    }
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_EXTENDED_SYS_STATE), 2);
}

TEST_F(MavlinkTelemetryTest, RequestMessageSendsDisabledStreamMessage)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_ATTITUDE), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_ATTITUDE), 1);
    sent.clear();
    tick(2000);
    EXPECT_EQ(countOf(MAVLINK_MSG_ID_ATTITUDE), 0);
}

TEST_F(MavlinkTelemetryTest, RequestMessageHomePosition)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_HOME_POSITION), MAV_RESULT_FAILED);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_HOME_POSITION), nullptr);

    GPS_home_llh.lat = -335000000;
    GPS_home_llh.lon = 1512500000;
    GPS_home_llh.altCm = 4200;
    ENABLE_STATE(GPS_FIX_HOME);
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_HOME_POSITION), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_HOME_POSITION);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(mavlink_msg_home_position_get_latitude(m), -335000000);
    EXPECT_EQ(mavlink_msg_home_position_get_altitude(m), 42000);
}

TEST_F(MavlinkTelemetryTest, RequestMessageGpsGlobalOrigin)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN), MAV_RESULT_FAILED);
    EXPECT_EQ(lastOf(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN), nullptr);

    GPS_home_llh.lat = -335000000;
    GPS_home_llh.altCm = 4200;
    ENABLE_STATE(GPS_FIX_HOME);
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN);
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(mavlink_msg_gps_global_origin_get_latitude(m), -335000000);
    EXPECT_EQ(mavlink_msg_gps_global_origin_get_altitude(m), 42000);
}

TEST_F(MavlinkTelemetryTest, RequestMessageSystemTime)
{
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_SYSTEM_TIME), MAV_RESULT_ACCEPTED);
    EXPECT_NE(lastOf(MAVLINK_MSG_ID_SYSTEM_TIME), nullptr);
}

TEST_F(MavlinkTelemetryTest, AttitudeReportsBodyRates)
{
    gyro.gyroADCf[FD_ROLL] = 90.0f;
    gyro.gyroADCf[FD_PITCH] = 45.0f;
    gyro.gyroADCf[FD_YAW] = 30.0f;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_ATTITUDE), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_ATTITUDE);
    ASSERT_NE(m, nullptr);
    EXPECT_NEAR(mavlink_msg_attitude_get_rollspeed(m), M_PI / 2, 1e-4);
    EXPECT_NEAR(mavlink_msg_attitude_get_pitchspeed(m), -M_PI / 4, 1e-4);
    EXPECT_NEAR(mavlink_msg_attitude_get_yawspeed(m), -M_PI / 6, 1e-4);
}

TEST_F(MavlinkTelemetryTest, VfrHudReportsAmslAltitudeClimbAndMotorThrottle)
{
    sensorMask = SENSOR_GPS;
    gpsSol.llh.altCm = 4250;
    gpsSol.groundSpeed = 350;
    altitudeDerivative = 150.0f;
    for (int i = 0; i < motorCount; i++) {
        motor[i] = 1400.0f;
    }

    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    const mavlink_message_t *m = lastOf(MAVLINK_MSG_ID_VFR_HUD);
    ASSERT_NE(m, nullptr);
    EXPECT_FLOAT_EQ(mavlink_msg_vfr_hud_get_alt(m), 42.5f);
    EXPECT_FLOAT_EQ(mavlink_msg_vfr_hud_get_groundspeed(m), 3.5f);
    EXPECT_FLOAT_EQ(mavlink_msg_vfr_hud_get_climb(m), 1.5f);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(m), 0);

    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 40);
}

TEST_F(MavlinkTelemetryTest, VfrHudThrottleFollowsMotorOutputs)
{
    ENABLE_ARMING_FLAG(ARMED);

    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 0);

    motor[0] = 2100.0f;
    motor[1] = 2000.0f;
    motor[2] = 1600.0f;
    motor[3] = 900.0f;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 65);
}

TEST_F(MavlinkTelemetryTest, VfrHudThrottleStoppedMotorIsZero)
{
    ENABLE_ARMING_FLAG(ARMED);
    motorStopped = 1460.0f;
    for (int i = 0; i < motorCount; i++) {
        motor[i] = 1460.0f;
    }
    featureMask = FEATURE_3D;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 0);

    featureMask = 0;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 0);
}

TEST_F(MavlinkTelemetryTest, VfrHudThrottleFullDriveIsNotStopped)
{
    ENABLE_ARMING_FLAG(ARMED);
    motorStopped = 1000.0f;
    for (int i = 0; i < motorCount; i++) {
        motor[i] = 2000.0f;
    }
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 100);
}

TEST_F(MavlinkTelemetryTest, VfrHudThrottleReversibleCountsBothDirections)
{
    ENABLE_ARMING_FLAG(ARMED);
    featureMask = FEATURE_3D;
    motor[0] = 1000.0f;
    motor[1] = 2000.0f;
    motor[2] = 1250.0f;
    motor[3] = 1750.0f;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_VFR_HUD), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_vfr_hud_get_throttle(lastOf(MAVLINK_MSG_ID_VFR_HUD)), 75);
}

TEST_F(MavlinkTelemetryTest, GpsRawFixTypeIgnoresSatelliteCount)
{
    sensorMask = SENSOR_GPS;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GPS_RAW_INT), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_gps_raw_int_get_fix_type(lastOf(MAVLINK_MSG_ID_GPS_RAW_INT)), GPS_FIX_TYPE_NO_FIX);

    ENABLE_STATE(GPS_FIX);
    gpsSol.numSat = 3;
    EXPECT_EQ(ackResultFor(MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_GPS_RAW_INT), MAV_RESULT_ACCEPTED);
    EXPECT_EQ(mavlink_msg_gps_raw_int_get_fix_type(lastOf(MAVLINK_MSG_ID_GPS_RAW_INT)), GPS_FIX_TYPE_3D_FIX);
}
