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
#include <string.h>
#include <math.h>

//#define DEBUG_MSP

extern "C" {
    #include <platform.h>
    #include "build/build_config.h"
    #include "build/version.h"
    #include "build/debug.h"

    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"
    #include "common/streambuf.h"
    #include "common/utils.h"

    #include "config/parameter_group.h"
    #include "config/config_eeprom.h"

    #include "drivers/system.h"
    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/serial.h"
    #include "drivers/serial_softserial.h"
    #include "drivers/buf_writer.h"

    #include "rx/rx.h"

    #include "fc/rc_controls.h"
    #include "fc/rate_profile.h"
    #include "fc/rc_adjustments.h"

    #include "io/gps.h"
    #include "io/gimbal.h"
    #include "io/ledstrip.h"
    #include "io/motor_and_servo.h"
    #include "io/transponder_ir.h"
    #include "io/serial.h"

    #include "msp/msp_protocol.h"
    #include "msp/msp.h"
    #include "msp/msp_serial.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"

    #include "sensors/sensors.h"
    #include "sensors/boardalignment.h"
    #include "sensors/battery.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"

    #include "flight/mixer.h"
    #include "flight/servos.h"
    #include "flight/pid.h"
    #include "flight/navigation.h"
    #include "flight/imu.h"
    #include "flight/failsafe.h"

    #include "config/parameter_group_ids.h"
    #include "fc/runtime_config.h"
    #include "config/profile.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);
    PG_REGISTER(sensorAlignmentConfig_t, sensorAlignmentConfig, PG_SENSOR_ALIGNMENT_CONFIG, 0);
    PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
    PG_REGISTER(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 0);
    PG_REGISTER(transponderConfig_t, transponderConfig, PG_TRANSPONDER_CONFIG, 0);
    PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
    PG_REGISTER(boardAlignment_t, boardAlignment, PG_BOARD_ALIGNMENT, 0);
    PG_REGISTER_ARR(servoMixer_t, MAX_SERVO_RULES, customServoMixer, PG_SERVO_MIXER, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
    PG_REGISTER(imuConfig_t, imuConfig, PG_IMU_CONFIG, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER_ARR(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, failsafeChannelConfigs, PG_FAILSAFE_CHANNEL_CONFIG, 0);
    PG_REGISTER_ARR(rxChannelRangeConfiguration_t, NON_AUX_CHANNEL_COUNT, channelRanges, PG_CHANNEL_RANGE_CONFIG, 0);
    PG_REGISTER(motor3DConfig_t, motor3DConfig, PG_MOTOR_3D_CONFIG, 0);
    PG_REGISTER_ARR(ledConfig_t, LED_MAX_STRIP_LENGTH, ledConfigs, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER_ARR(hsvColor_t, LED_CONFIGURABLE_COLOR_COUNT, colors, PG_COLOR_CONFIG, 0);
    PG_REGISTER_ARR(modeColorIndexes_t, LED_MODE_COUNT, modeColors, PG_MODE_COLOR_CONFIG, 0);
    PG_REGISTER_ARR(specialColorIndexes_t, 1, specialColors, PG_SPECIAL_COLOR_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(frskyTelemetryConfig_t, frskyTelemetryConfig, PG_FRSKY_TELEMETRY_CONFIG, 0);
    PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

    PG_REGISTER_PROFILE_WITH_RESET_FN(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);
    void pgResetFn_pidProfile(pidProfile_t *) {}

    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER_PROFILE(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER_PROFILE(adjustmentProfile_t, adjustmentProfile, PG_ADJUSTMENT_PROFILE, 0);
    PG_REGISTER_PROFILE(compassConfig_t, compassConfig, PG_COMPASS_CONFIGURATION, 0);
    PG_REGISTER_PROFILE(modeActivationProfile_t, modeActivationProfile, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER_PROFILE(servoProfile_t, servoProfile, PG_SERVO_PROFILE, 0);
}


#define MSP_BUFFER_SIZE 256
#define BARRIER "Memory barrier!"  // 15 bytes + \0

class MspTest : public ::testing::Test {
protected:
    mspPacket_t cmd, reply;
    uint8_t rbuf[MSP_BUFFER_SIZE];
    char barrier_r[16];                    // check for buffer overflow
    uint8_t sbuf[MSP_BUFFER_SIZE];
    char barrier_s[16];
    void resetReply() {
        memset(rbuf, 0xde, sizeof(rbuf));
        reply.buf.ptr = rbuf;
        reply.buf.end = ARRAYEND(rbuf);       // whole buffer available
    }
    void resetCmd() {
        memset(sbuf, 0xad, sizeof(sbuf));
        cmd.buf.ptr = sbuf;
        cmd.buf.end = sbuf;                   // command buffer is empty by default
    }
    void resetPackets() {
        resetCmd();
        resetReply();
    }
    void copyReplyDataToCmd() {
        resetCmd();                           // cleanup first
        // copy previously received data to command buffer
        memcpy(sbuf, rbuf, reply.buf.ptr - rbuf);
        cmd.buf.ptr = sbuf;
        cmd.buf.end = sbuf + (reply.buf.ptr - rbuf);
    }
    virtual void SetUp() {
        strcpy(barrier_r, BARRIER);
        strcpy(barrier_s, BARRIER);
        resetPackets();
    }
    virtual void TearDown() {
        EXPECT_STREQ(barrier_r, BARRIER);
        EXPECT_STREQ(barrier_s, BARRIER);
    }
};

TEST_F(MspTest, TestMsp_API_VERSION)
{
    cmd.cmd = MSP_API_VERSION;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(3, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_API_VERSION, reply.cmd);
    EXPECT_EQ(MSP_PROTOCOL_VERSION, rbuf[0]);
    EXPECT_EQ(API_VERSION_MAJOR, rbuf[1]);
    EXPECT_EQ(API_VERSION_MINOR, rbuf[2]);
}

TEST_F(MspTest, TestMsp_FC_VARIANT)
{
    cmd.cmd = MSP_FC_VARIANT;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(FLIGHT_CONTROLLER_IDENTIFIER_LENGTH, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_FC_VARIANT, reply.cmd);
    EXPECT_EQ('C', rbuf[0]);
    EXPECT_EQ('L', rbuf[1]);
    EXPECT_EQ('F', rbuf[2]);
    EXPECT_EQ('L', rbuf[3]);
}

TEST_F(MspTest, TestMsp_FC_VERSION)
{
    cmd.cmd = MSP_FC_VERSION;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(FLIGHT_CONTROLLER_VERSION_LENGTH, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_FC_VERSION, reply.cmd);
    EXPECT_EQ(FC_VERSION_MAJOR, rbuf[0]);
    EXPECT_EQ(FC_VERSION_MINOR, rbuf[1]);
    EXPECT_EQ(FC_VERSION_PATCH_LEVEL, rbuf[2]);
}

TEST_F(MspTest, TestMsp_PID_CONTROLLER)
{
    pgActivateProfile(0);
    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;

    cmd.cmd = MSP_PID_CONTROLLER;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(1, reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_PID_CONTROLLER, reply.cmd);
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, rbuf[0]);
}

TEST_F(MspTest, TestMsp_SET_PID_CONTROLLER)
{
    // set the pidController to a different value so we can check if it gets read back properly
    pidProfile()->pidController = PID_CONTROLLER_LUX_FLOAT;

    cmd.cmd = MSP_SET_PID_CONTROLLER;
    *cmd.buf.end++ = PID_CONTROLLER_MWREWRITE;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, pidProfile()->pidController);
}

TEST_F(MspTest, TestMsp_PID)
{
    // check the buffer is big enough for the data to read in
    EXPECT_LE(3 * PID_ITEM_COUNT, MSP_BUFFER_SIZE);
    // set up some test data
    const int P8_ROLL = 40;
    const int I8_ROLL = 30;
    const int D8_ROLL = 23;
    const int P8_PITCH = 41;
    const int I8_PITCH = 31;
    const int D8_PITCH = 24;
    const int P8_YAW = 85;
    const int I8_YAW = 45;
    const int D8_YAW = 1;
    const int P8_PIDALT = 50;
    const int I8_PIDALT = 2;
    const int D8_PIDALT = 3;
    const int P8_PIDPOS = 15; // POSHOLD_P * 100;
    const int I8_PIDPOS = 4; // POSHOLD_I * 100;
    const int D8_PIDPOS = 5;
    const int P8_PIDPOSR = 34; // POSHOLD_RATE_P * 10;
    const int I8_PIDPOSR = 14; // POSHOLD_RATE_I * 100;
    const int D8_PIDPOSR = 53; // POSHOLD_RATE_D * 1000;
    const int P8_PIDNAVR = 25; // NAV_P * 10;
    const int I8_PIDNAVR = 33; // NAV_I * 100;
    const int D8_PIDNAVR = 83; // NAV_D * 1000;
    const int P8_PIDLEVEL = 90;
    const int I8_PIDLEVEL = 10;
    const int D8_PIDLEVEL = 100;
    const int P8_PIDMAG = 40;
    const int P8_PIDVEL = 120;
    const int I8_PIDVEL = 45;
    const int D8_PIDVEL = 7;

    pgActivateProfile(0);

    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;
    pidProfile()->P8[PIDROLL] = P8_ROLL;
    pidProfile()->I8[PIDROLL] = I8_ROLL;
    pidProfile()->D8[PIDROLL] = D8_ROLL;
    pidProfile()->P8[PIDPITCH] = P8_PITCH;
    pidProfile()->I8[PIDPITCH] = I8_PITCH;
    pidProfile()->D8[PIDPITCH] = D8_PITCH;
    pidProfile()->P8[PIDYAW] = P8_YAW;
    pidProfile()->I8[PIDYAW] = I8_YAW;
    pidProfile()->D8[PIDYAW] = D8_YAW;
    pidProfile()->P8[PIDALT] = P8_PIDALT;
    pidProfile()->I8[PIDALT] = I8_PIDALT;
    pidProfile()->D8[PIDALT] = D8_PIDALT;
    pidProfile()->P8[PIDPOS] = P8_PIDPOS;
    pidProfile()->I8[PIDPOS] = I8_PIDPOS;
    pidProfile()->D8[PIDPOS] = D8_PIDPOS;
    pidProfile()->P8[PIDPOSR] = P8_PIDPOSR;
    pidProfile()->I8[PIDPOSR] = I8_PIDPOSR;
    pidProfile()->D8[PIDPOSR] = D8_PIDPOSR;
    pidProfile()->P8[PIDNAVR] = P8_PIDNAVR;
    pidProfile()->I8[PIDNAVR] = I8_PIDNAVR;
    pidProfile()->D8[PIDNAVR] = D8_PIDNAVR;
    pidProfile()->P8[PIDLEVEL] = P8_PIDLEVEL;
    pidProfile()->I8[PIDLEVEL] = I8_PIDLEVEL;
    pidProfile()->D8[PIDLEVEL] = D8_PIDLEVEL;
    pidProfile()->P8[PIDMAG] = P8_PIDMAG;
    pidProfile()->P8[PIDVEL] = P8_PIDVEL;
    pidProfile()->I8[PIDVEL] = I8_PIDVEL;
    pidProfile()->D8[PIDVEL] = D8_PIDVEL;

    // use the MSP to write out the PID values
    cmd.cmd = MSP_PID;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(3 * PID_ITEM_COUNT, reply.buf.ptr - rbuf) << "Reply size";
    // check few values, just to make sure they have been written correctly
    EXPECT_EQ(P8_YAW, rbuf[6]);
    EXPECT_EQ(I8_YAW, rbuf[7]);
    EXPECT_EQ(D8_YAW, rbuf[8]);

    // reset test values to zero, so we can check if they get read properly
    memset(pidProfile(), 0, sizeof(*pidProfile()));

    // now use the MSP to set the PID values and check they are the same as written
    cmd.cmd = MSP_SET_PID;
    copyReplyDataToCmd();
    resetReply();

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    // check the values are as expected
    EXPECT_EQ(P8_ROLL, pidProfile()->P8[PIDROLL]);
    EXPECT_EQ(I8_ROLL, pidProfile()->I8[PIDROLL]);
    EXPECT_EQ(D8_ROLL, pidProfile()->D8[PIDROLL]);
    EXPECT_EQ(P8_PITCH, pidProfile()->P8[PIDPITCH]);
    EXPECT_EQ(I8_PITCH, pidProfile()->I8[PIDPITCH]);
    EXPECT_EQ(D8_PITCH, pidProfile()->D8[PIDPITCH]);
    EXPECT_EQ(P8_YAW, pidProfile()->P8[PIDYAW]);
    EXPECT_EQ(I8_YAW, pidProfile()->I8[PIDYAW]);
    EXPECT_EQ(D8_YAW, pidProfile()->D8[PIDYAW]);
    EXPECT_EQ(P8_PIDALT, pidProfile()->P8[PIDALT]);
    EXPECT_EQ(I8_PIDALT, pidProfile()->I8[PIDALT]);
    EXPECT_EQ(D8_PIDALT, pidProfile()->D8[PIDALT]);
    EXPECT_EQ(P8_PIDPOS, pidProfile()->P8[PIDPOS]);
    EXPECT_EQ(I8_PIDPOS, pidProfile()->I8[PIDPOS]);
    EXPECT_EQ(D8_PIDPOS, pidProfile()->D8[PIDPOS]);
    EXPECT_EQ(P8_PIDPOSR, pidProfile()->P8[PIDPOSR]);
    EXPECT_EQ(I8_PIDPOSR, pidProfile()->I8[PIDPOSR]);
    EXPECT_EQ(D8_PIDPOSR, pidProfile()->D8[PIDPOSR]);
    EXPECT_EQ(P8_PIDNAVR, pidProfile()->P8[PIDNAVR]);
    EXPECT_EQ(I8_PIDNAVR, pidProfile()->I8[PIDNAVR]);
    EXPECT_EQ(D8_PIDNAVR, pidProfile()->D8[PIDNAVR]);
    EXPECT_EQ(P8_PIDLEVEL, pidProfile()->P8[PIDLEVEL]);
    EXPECT_EQ(I8_PIDLEVEL, pidProfile()->I8[PIDLEVEL]);
    EXPECT_EQ(D8_PIDLEVEL, pidProfile()->D8[PIDLEVEL]);
    EXPECT_EQ(P8_PIDMAG, pidProfile()->P8[PIDMAG]);
    EXPECT_EQ(P8_PIDVEL, pidProfile()->P8[PIDVEL]);
    EXPECT_EQ(I8_PIDVEL, pidProfile()->I8[PIDVEL]);
    EXPECT_EQ(D8_PIDVEL, pidProfile()->D8[PIDVEL]);
}


TEST_F(MspTest, TestMsp_BOARD_ALIGNMENT)
{
    const boardAlignment_t testBoardAlignment = {295, 147, -202};

    *boardAlignment() = testBoardAlignment;
    // use the MSP to write out the test values

    cmd.cmd = MSP_BOARD_ALIGNMENT;

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    EXPECT_EQ(sizeof(boardAlignment_t), reply.buf.ptr - rbuf) << "Reply size";
    EXPECT_EQ(MSP_BOARD_ALIGNMENT, reply.cmd);
    EXPECT_EQ(testBoardAlignment.rollDegrees & 0xff, rbuf[0]);
    EXPECT_EQ(testBoardAlignment.rollDegrees >> 8, rbuf[1]);

    // reset test values to zero, so we can check if they get read properly
    memset(boardAlignment(), 0, sizeof(*boardAlignment()));

    // now use the MSP to set the values and check they are the same
    cmd.cmd = MSP_SET_BOARD_ALIGNMENT;
    copyReplyDataToCmd();
    resetReply();

    EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

    // check the values are as expected
    EXPECT_FLOAT_EQ(testBoardAlignment.rollDegrees, boardAlignment()->rollDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.pitchDegrees, boardAlignment()->pitchDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.yawDegrees, boardAlignment()->yawDegrees);
}

TEST_F(MspTest, TestMspCommands)
{

    static const uint8_t outMessages[] = {
        MSP_API_VERSION,                // 1    //out message
        MSP_FC_VARIANT,                 // 2    //out message
        MSP_FC_VERSION,                 // 3    //out message
        MSP_BOARD_INFO,                 // 4    //out message
        MSP_BUILD_INFO,                 // 5    //out message
        // MSP commands for Cleanflight original features
        MSP_MODE_RANGES,                // 34    //out message         Returns all mode ranges
        MSP_FEATURE,                    // 36
        MSP_BOARD_ALIGNMENT,            // 38
        MSP_CURRENT_METER_CONFIG,       // 40
        MSP_MIXER,                      // 42
        MSP_RX_CONFIG,                  // 44
        MSP_LED_COLORS,                 // 46
        MSP_LED_STRIP_CONFIG,           // 48
        MSP_RSSI_CONFIG,                // 50
        MSP_ADJUSTMENT_RANGES,          // 52
        // private - only to be used by the configurator, the commands are likely to change
//!! not tested        MSP_CF_SERIAL_CONFIG,           // 54
        MSP_VOLTAGE_METER_CONFIG,       // 56
        MSP_SONAR_ALTITUDE,             // 58 //out message get sonar altitude [cm]
        MSP_PID_CONTROLLER,             // 59
        MSP_ARMING_CONFIG,              // 61 //out message         Returns auto_disarm_delay and disarm_kill_switch parameters
        MSP_DATAFLASH_SUMMARY,          // 70 //out message - get description of dataflash chip
//!! not tested       MSP_DATAFLASH_READ,             // 71 //out message - get content of dataflash chip
        MSP_LOOP_TIME,                  // 73 //out message         Returns FC cycle time i.e looptime parameter
        MSP_FAILSAFE_CONFIG,            // 75 //out message         Returns FC Fail-Safe settings
        MSP_RXFAIL_CONFIG,              // 77 //out message         Returns RXFAIL settings
        MSP_SDCARD_SUMMARY,             // 79 //out message         Get the state of the SD card
        MSP_BLACKBOX_CONFIG,            // 80 //out message         Get blackbox settings
        MSP_TRANSPONDER_CONFIG,         // 82 //out message         Get transponder settings
        MSP_LED_STRIP_MODECOLOR,        // 85 //out message         Get LED strip mode_color settings
        MSP_SET_LED_STRIP_MODECOLOR,    // 86 //out message         Set LED strip mode_color settings
        // Baseflight MSP commands (if enabled they exist in Cleanflight)
        MSP_RX_MAP,                     // 64 //out message get channel map (also returns number of channels total)
        // DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
        MSP_BF_CONFIG,                  // 66 //out message baseflight-specific settings that aren't covered elsewhere
        // DEPRECATED - Use MSP_BUILD_INFO instead
        MSP_BF_BUILD_INFO,              // 69 //out message build date as well as some space for future expansion
        // Multwii original MSP commands
        // DEPRECATED - See MSP_API_VERSION and MSP_MIXER
        MSP_IDENT,               // 100    //out message         mixerMode + multiwii version + protocol version + capability variable
        MSP_STATUS,              // 101    //out message         cycletime & errors_count & sensor present & box activation & current setting number
        MSP_RAW_IMU,             // 102    //out message         9 DOF
        MSP_SERVO,               // 103    //out message         servos
        MSP_MOTOR,               // 104    //out message         motors
        MSP_RC,                  // 105    //out message         rc channels and more
        MSP_RAW_GPS,             // 106    //out message         fix, numsat, lat, lon, alt, speed, ground course
        MSP_COMP_GPS,            // 107    //out message         distance home, direction home
        MSP_ATTITUDE,            // 108    //out message         2 angles 1 heading
        MSP_ALTITUDE,            // 109    //out message         altitude, variometer
        MSP_ANALOG,              // 110    //out message         vbat, powermetersum, rssi if available on RX
        MSP_RC_TUNING,           // 111    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
        MSP_PID,                 // 112    //out message         P I D coeff (9 are used currently)
//!! not implemented in serial_msp.c      MSP_BOX,                 // 113    //out message         BOX setup (number is dependant of your setup)
        MSP_MISC,                // 114    //out message         powermeter trig
        MSP_MOTOR_PINS,          // 115    //out message         which pins are in use for motors & servos, for GUI
        MSP_BOXNAMES,            // 116    //out message         the aux switch names
        MSP_PIDNAMES,            // 117    //out message         the PID names
        MSP_WP,                  // 118    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
        MSP_BOXIDS,              // 119    //out message         get the permanent IDs associated to BOXes
        MSP_SERVO_CONFIGURATIONS,// 120    //out message         All servo configurations.
//!! not implemented in serial_msp.c       MSP_NAV_STATUS,          // 121    //out message         Returns navigation status
//!! not implemented in serial_msp.c       MSP_NAV_CONFIG,          // 122    //out message         Returns navigation parameters
        MSP_3D,                  // 124    //out message         Settings needed for reversible ESCs
        MSP_RC_DEADBAND,         // 125    //out message         deadbands for yaw alt pitch roll
        MSP_SENSOR_ALIGNMENT,    // 126    //out message         orientation of acc,gyro,mag
//!! not implemented in serial_msp.c       MSP_DEBUGMSG,            // 253    //out message         debug string buffer
        MSP_DEBUG,               // 254    //out message         debug1,debug2,debug3,debug4
        // Additional commands that are not compatible with MultiWii
        MSP_STATUS_EX,           // 150    //out message         cycletime, errors_count, CPU load, sensor present etc
        MSP_UID,                 // 160    //out message         Unique device ID
        MSP_GPSSVINFO,           // 164    //out message         get Signal Strength (only U-Blox)
        MSP_ACC_TRIM,            // 240    //out message         get acc angle trim values
        MSP_SERVO_MIX_RULES,     // 241    //out message         Returns servo mixer configuration
    };
    for (uint ii = 0; ii < ARRAYLEN(outMessages); ii++) {
        resetPackets();
        cmd.cmd = outMessages[ii];

        EXPECT_GT(mspProcessCommand(&cmd, &reply), 0);

        EXPECT_EQ(outMessages[ii], reply.cmd) << "Command index " << ii;
        EXPECT_LT(0, reply.result) << "Command index " << ii;
    }
}

// STUBS
extern "C" {
//
mspPostProcessFuncPtr mspPostProcessFn = NULL;
// from acceleration.c
acc_t acc;                       // acc access functions
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired) {UNUSED(calibrationCyclesRequired);}
// from altitudehold.c
int32_t AltHold;
int32_t vario = 0;                      // variometer in cm/s
int32_t altitudeHoldGetEstimatedAltitude(void) {return 0;}
// from battery.c
uint16_t vbat = 0;                   // battery voltage in 0.1V steps (filtered)
int32_t amperage = 0;               // amperage read by current sensor in centiampere (1/100th A)
int32_t mAhDrawn = 0;               // milliampere hours drawn from the battery since start
// from compass.c
int32_t magADC[XYZ_AXIS_COUNT];
// from config.c
controlRateConfig_t controlRateProfile;
controlRateConfig_t *currentControlRateProfile = &controlRateProfile;
void resetPidProfile(pidProfile_t *pidProfile) {UNUSED(pidProfile);}
void handleOneshotFeatureChangeOnRestart(void) {}
void readEEPROM(void) {}
void resetEEPROM(void) {}
void writeEEPROM(void) {}
void changeProfile(uint8_t) {};
void setProfile(uint8_t) {};
uint8_t getCurrentProfile(void) { return 0; };
bool feature(uint32_t mask) {UNUSED(mask);return false;}
void featureSet(uint32_t mask) {UNUSED(mask);}
void featureClearAll() {}
uint32_t featureMask(void) {return 0;}
// from debug.c
int16_t debug[DEBUG16_VALUE_COUNT];
// from gps.c
#define GPS_SV_MAXSATS   16
int32_t GPS_coord[2];               // LAT/LON
uint8_t GPS_numSat;
uint8_t GPS_update = 0;             // it's a binary toggle to distinct a GPS position update
uint16_t GPS_altitude;              // altitude in 0.1m
uint16_t GPS_speed;                 // speed in 0.1m/s
uint16_t GPS_ground_course = 0;     // degrees * 10
uint8_t GPS_numCh;                          // Number of channels
uint8_t GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)
// from gyro.c
int32_t gyroADC[XYZ_AXIS_COUNT];
// form imu.c
attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
int16_t accSmooth[XYZ_AXIS_COUNT];
// from ledstrip.c
void reevalulateLedConfig(void) {}
bool setModeColor(ledModeIndex_e , int , int ) { return true; }
// from mixer.c
int16_t motor[MAX_SUPPORTED_MOTORS];
int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
int16_t servo[MAX_SUPPORTED_SERVOS];
void stopMotors(void) {}
void loadCustomServoMixer(void) {}
// from msp.c
void rxMspFrameReceive(uint16_t *, int ) {}
// from mw.c
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
int16_t magHold;
// from navigation.c
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
navigationMode_e nav_mode = NAV_MODE_NONE;    // Navigation mode
void GPS_set_next_wp(int32_t *, int32_t *) {}
// from pid.c
void pidSetController(pidControllerType_e) {}
// from rc_controls.c
uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e
bool rcModeIsActive(boxId_e modeId) { return rcModeActivationMask & (1 << modeId); }
void useRcControlsConfig(modeActivationCondition_t *) {};
// from runtime_config.c
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint16_t disableFlightMode(flightModeFlags_e) { return 0; }
bool sensors(uint32_t mask) {UNUSED(mask);return 0;}
// from rx.c
uint16_t rssi = 0;                  // range: [0;1023]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
rxRuntimeConfig_t rxRuntimeConfig;
// from system.c
void delay(uint32_t ms) {UNUSED(ms);}
// from system_stm32fN0x.c
void systemReset(void) {}
void systemResetToBootloader(void) {}
// from scheduler.c
uint16_t averageSystemLoadPercent = 0;
// from transponder_ir.c
void transponderUpdateData(uint8_t*) {}
// from serial port drivers
serialPort_t *usbVcpOpen(void) { return NULL; }
serialPort_t *uartOpen(USART_TypeDef *, serialReceiveCallbackPtr, uint32_t, portMode_t, portOptions_t) { return NULL; }
serialPort_t *openSoftSerial(softSerialPortIndex_e, serialReceiveCallbackPtr, uint32_t, portOptions_t) { return NULL; }
void serialSetMode(serialPort_t *, portMode_t) {}

void mspSerialProcess() {}
int mspClientProcessInCommand(mspPacket_t *) { return false; }
bool isSerialTransmitBufferEmpty(serialPort_t *) { return true; }
}

