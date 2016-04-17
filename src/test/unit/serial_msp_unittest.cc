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
    #include "build_config.h"
    #include "version.h"
    #include "debug.h"

    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"

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

    #include "io/rc_controls.h"
    #include "io/rate_profile.h"
    #include "io/rc_adjustments.h"
    #include "io/gps.h"
    #include "io/gimbal.h"
    #include "io/ledstrip.h"
    #include "io/msp_protocol.h"
    #include "io/serial_msp.h"
    #include "io/motor_and_servo.h"
    #include "io/transponder_ir.h"

    #include "telemetry/telemetry.h"
    #include "telemetry/frsky.h"

    #include "sensors/sensors.h"
    #include "sensors/boardalignment.h"
    #include "sensors/battery.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/navigation.h"
    #include "flight/imu.h"
    #include "flight/failsafe.h"

    #include "config/parameter_group_ids.h"
    #include "config/runtime_config.h"
    #include "config/config.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    void setCurrentPort(mspPort_t *port);
    uint8_t pgMatcherForMSPSet(const pgRegistry_t *candidate, const void *criteria);
    uint8_t pgMatcherForMSP(const pgRegistry_t *candidate, const void *criteria);
    void mspProcessReceivedCommand();
    extern mspPort_t *currentPort;
    extern bufWriter_t *writer;
    extern mspPort_t mspPorts[];

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
    PG_REGISTER_ARR(ledConfig_t, MAX_LED_STRIP_LENGTH, ledConfigs, PG_LED_STRIP_CONFIG, 0);
    PG_REGISTER_ARR(hsvColor_t, CONFIGURABLE_COLOR_COUNT, colors, PG_COLOR_CONFIG, 0);
    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(telemetryConfig_t, telemetryConfig, PG_TELEMETRY_CONFIG, 0);
    PG_REGISTER(frskyTelemetryConfig_t, frskyTelemetryConfig, PG_FRSKY_TELEMETRY_CONFIG, 0);

    PG_REGISTER_PROFILE_WITH_RESET_FN(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);
    void pgResetFn_pidProfile(pidProfile_t *) {}

    PG_REGISTER_PROFILE(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);
    PG_REGISTER_PROFILE(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER_PROFILE(adjustmentProfile_t, adjustmentProfile, PG_ADJUSTMENT_PROFILE, 0);
    PG_REGISTER_PROFILE(compassConfig_t, compassConfig, PG_COMPASS_CONFIGURATION, 0);
    PG_REGISTER_PROFILE(modeActivationProfile_t, modeActivationProfile, PG_MODE_ACTIVATION_PROFILE, 0);
    PG_REGISTER_PROFILE(servoProfile_t, servoProfile, PG_SERVO_PROFILE, 0);
}

typedef struct mspHeader_s {
    uint8_t dollar;
    uint8_t m;
    uint8_t direction;
    uint8_t size;
    uint8_t type;
} mspHeader_t;

typedef struct mspResonse_s {
    mspHeader_t header;
    uint8_t payload[];
} mspResponse_t;

#define SERIAL_BUFFER_SIZE 256
typedef union mspBuffer_u {
    mspResponse_t mspResponse;
    uint8_t buf[SERIAL_BUFFER_SIZE];
} mspBuffer_t;

static mspBuffer_t serialBuffer;

static int serialWritePos = 0;
static int serialReadPos = 0;

uint8_t buf[sizeof(bufWriter_t) + SERIAL_BUFFER_SIZE];

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    UNUSED(instance);
    serialBuffer.buf[serialWritePos] = ch;
    ++serialWritePos;
}

void serialWriteBufShim(void *instance, uint8_t *data, int count)
{
    for (uint8_t *p = data; count > 0; count--, p++) {
        serialWrite((serialPort_t *)instance, *p);
    }
}

void serialBeginWrite(serialPort_t *instance)
{
    UNUSED(instance);
}

void serialEndWrite(serialPort_t *instance)
{
    UNUSED(instance);
}

uint8_t serialRxBytesWaiting(serialPort_t *instance)
{
    UNUSED(instance);
    if (serialWritePos > serialReadPos) {
        return serialWritePos - serialReadPos;
    } else {
        return 0;
    }
}

uint8_t serialRead(serialPort_t *instance)
{
    UNUSED(instance);
    const uint8_t ch = serialBuffer.buf[serialReadPos];
    ++serialReadPos;
    if (currentPort->indRX == MSP_PORT_INBUF_SIZE) {
        currentPort->indRX = 0;
    }
    currentPort->inBuf[currentPort->indRX] = ch;
    ++currentPort->indRX;
    return ch;
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

class SerialMspUnitTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        memset(serialBuffer.buf, 0, sizeof(serialBuffer));
        setCurrentPort(&mspPorts[0]);
        writer = bufWriterInit(buf, sizeof(buf), (bufWrite_t)serialWriteBufShim, &mspPorts[0]);
    }
};


TEST_F(SerialMspUnitTest, TestMspProcessReceivedCommand)
{
    // check the MSP_API_VERSION is written out correctly
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_API_VERSION;
    mspProcessReceivedCommand();

    EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
    EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
    EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
    EXPECT_EQ(3, serialBuffer.mspResponse.header.size);
    EXPECT_EQ(MSP_API_VERSION, serialBuffer.mspResponse.header.type);
    EXPECT_EQ(MSP_PROTOCOL_VERSION, serialBuffer.mspResponse.payload[0]);
    EXPECT_EQ(API_VERSION_MAJOR, serialBuffer.mspResponse.payload[1]);
    EXPECT_EQ(API_VERSION_MINOR, serialBuffer.mspResponse.payload[2]);
    int checksum = 3 ^ MSP_API_VERSION;
    checksum ^= MSP_PROTOCOL_VERSION ^ API_VERSION_MAJOR ^ API_VERSION_MINOR;
    EXPECT_EQ(checksum, serialBuffer.mspResponse.payload[3]);// checksum

    // check the MSP_FC_VARIANT is written out correctly
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_FC_VARIANT;
    mspProcessReceivedCommand();

    EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
    EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
    EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
    EXPECT_EQ(FLIGHT_CONTROLLER_IDENTIFIER_LENGTH, serialBuffer.mspResponse.header.size);
    EXPECT_EQ(MSP_FC_VARIANT, serialBuffer.mspResponse.header.type);
    EXPECT_EQ('C', serialBuffer.mspResponse.payload[0]);
    EXPECT_EQ('L', serialBuffer.mspResponse.payload[1]);
    EXPECT_EQ('F', serialBuffer.mspResponse.payload[2]);
    EXPECT_EQ('L', serialBuffer.mspResponse.payload[3]);
    checksum = FLIGHT_CONTROLLER_IDENTIFIER_LENGTH ^ MSP_FC_VARIANT;
    checksum ^= 'C'^ 'L' ^ 'F' ^ 'L';
    EXPECT_EQ(checksum, serialBuffer.mspResponse.payload[4]);

    // check the MSP_FC_VERSION is written out correctly
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_FC_VERSION;
    mspProcessReceivedCommand();
    EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
    EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
    EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
    EXPECT_EQ(FLIGHT_CONTROLLER_VERSION_LENGTH, serialBuffer.mspResponse.header.size);
    EXPECT_EQ(MSP_FC_VERSION, serialBuffer.mspResponse.header.type);
    EXPECT_EQ(FC_VERSION_MAJOR, serialBuffer.mspResponse.payload[0]);
    EXPECT_EQ(FC_VERSION_MINOR, serialBuffer.mspResponse.payload[1]);
    EXPECT_EQ(FC_VERSION_PATCH_LEVEL, serialBuffer.mspResponse.payload[2]);
    checksum = FLIGHT_CONTROLLER_VERSION_LENGTH ^ MSP_FC_VERSION;
    checksum ^= FC_VERSION_MAJOR ^ FC_VERSION_MINOR ^ FC_VERSION_PATCH_LEVEL;
    EXPECT_EQ(checksum, serialBuffer.mspResponse.payload[3]);

    // check the MSP_PID_CONTROLLER is written out correctly
    serialWritePos = 0;
    serialReadPos = 0;
    pgActivateProfile(0);

    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;
    currentPort->cmdMSP = MSP_PID_CONTROLLER;
    mspProcessReceivedCommand();
    EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
    EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
    EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
    EXPECT_EQ(1, serialBuffer.mspResponse.header.size);
    EXPECT_EQ(MSP_PID_CONTROLLER, serialBuffer.mspResponse.header.type);
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, serialBuffer.mspResponse.payload[0]);
    checksum = 1 ^ MSP_PID_CONTROLLER ^ PID_CONTROLLER_MWREWRITE;
    EXPECT_EQ(checksum, serialBuffer.mspResponse.payload[1]);
}

TEST_F(SerialMspUnitTest, Test_PID_CONTROLLER)
{
    // Use the MSP to write out the PID values
    pgActivateProfile(0);

    pidProfile()->pidController = PID_CONTROLLER_MWREWRITE;
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_PID_CONTROLLER;
    mspProcessReceivedCommand();
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, serialBuffer.mspResponse.payload[0]);

    // set the pidController to a different value so we can check if it gets read back properly
    pidProfile()->pidController = PID_CONTROLLER_LUX_FLOAT;

    // Now use the MSP to read back the picController and check if it is the same
    // spoof a change from the written MSP_PID_CONTROLLER to the readable MSP_SET_PID_CONTROLLER
    currentPort->cmdMSP = MSP_SET_PID_CONTROLLER;
    serialBuffer.mspResponse.header.direction = '<';
    serialBuffer.mspResponse.header.type = currentPort->cmdMSP;
    // force the checksum
    serialBuffer.mspResponse.payload[1] ^= MSP_PID_CONTROLLER;
    serialBuffer.mspResponse.payload[1] ^= MSP_SET_PID_CONTROLLER;
    // copy the command data into the current port inBuf so it can be processed
    memcpy(currentPort->inBuf, serialBuffer.buf, MSP_PORT_INBUF_SIZE);

    // set the offset into the payload
    currentPort->indRX = offsetof(struct mspResonse_s, payload);
    mspProcessReceivedCommand();

    // check the pidController value has been read correctly
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, pidProfile()->pidController);
}

TEST_F(SerialMspUnitTest, Test_PIDValuesInt)
{
    // check the buffer is big enough for the data to read in
    EXPECT_LE(sizeof(mspHeader_t) + 3 * PID_ITEM_COUNT + 1, MSP_PORT_INBUF_SIZE); // +1 for checksum
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
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_PID;
    mspProcessReceivedCommand();
    EXPECT_EQ(3 * PID_ITEM_COUNT, serialBuffer.mspResponse.header.size);
    // check few values, just to make sure they have been written correctly
    EXPECT_EQ(P8_YAW, serialBuffer.mspResponse.payload[6]);
    EXPECT_EQ(I8_YAW, serialBuffer.mspResponse.payload[7]);
    EXPECT_EQ(D8_YAW, serialBuffer.mspResponse.payload[8]);

    // reset test values to zero, so we can check if they get read properly
    memset(pidProfile(), 0, sizeof(*pidProfile()));

    // now use the MSP to read back the PID values and check they are the same as written
    // spoof a change from the written MSP_PID to the readable MSP_SET_PID
    currentPort->cmdMSP = MSP_SET_PID;
    serialBuffer.mspResponse.header.direction = '<';
    serialBuffer.mspResponse.header.type = currentPort->cmdMSP;
    // force the checksum
    serialBuffer.mspResponse.payload[3 * PID_ITEM_COUNT] ^= MSP_PID;
    serialBuffer.mspResponse.payload[3 * PID_ITEM_COUNT] ^= MSP_SET_PID;
    // copy the command data into the current port inBuf so it can be processed
    memcpy(currentPort->inBuf, serialBuffer.buf, MSP_PORT_INBUF_SIZE);

    // set the offset into the payload
    currentPort->indRX = offsetof(struct mspResonse_s, payload);
    mspProcessReceivedCommand();

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


TEST_F(SerialMspUnitTest, Test_BoardAlignment)
{
    const uint8_t cmdMSP = MSP_BOARD_ALIGNMENT;
    const pgRegistry_t *reg = pgMatcher(pgMatcherForMSP, (void*)&cmdMSP);
    EXPECT_NE(static_cast<const pgRegistry_t*>(0), reg);
    EXPECT_EQ(reinterpret_cast<boardAlignment_t*>(reg->address), boardAlignment());

    const boardAlignment_t testBoardAlignment = {295, 147, -202};

    memcpy(boardAlignment(), &testBoardAlignment, sizeof(*boardAlignment()));
    // use the MSP to write out the test values
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_BOARD_ALIGNMENT;
    mspProcessReceivedCommand();
    EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
    EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
    EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
    EXPECT_EQ(sizeof(boardAlignment_t), serialBuffer.mspResponse.header.size);
    EXPECT_EQ(MSP_BOARD_ALIGNMENT, serialBuffer.mspResponse.header.type);
    EXPECT_EQ(testBoardAlignment.rollDegrees & 0xff, serialBuffer.mspResponse.payload[0]);
    EXPECT_EQ(testBoardAlignment.rollDegrees >> 8, serialBuffer.mspResponse.payload[1]);


    // reset test values to zero, so we can check if they get read properly
    memset(boardAlignment(), 0, sizeof(*boardAlignment()));

    // now use the MSP to read back the values and check they are the same
    // spoof a change from the written MSP_BOARD_ALIGNMENT to the readable MSP_SET_BOARD_ALIGNMENT
    currentPort->cmdMSP = MSP_SET_BOARD_ALIGNMENT;
    serialBuffer.mspResponse.header.direction = '<';
    serialBuffer.mspResponse.header.type = currentPort->cmdMSP;
    // force the checksum
    serialBuffer.mspResponse.payload[serialBuffer.mspResponse.header.size] ^= MSP_BOARD_ALIGNMENT;
    serialBuffer.mspResponse.payload[serialBuffer.mspResponse.header.size] ^= MSP_SET_BOARD_ALIGNMENT;
    // copy the command data into the current port inBuf so it can be processed
    memcpy(currentPort->inBuf, serialBuffer.buf, MSP_PORT_INBUF_SIZE);


    // set the offset into the payload
    currentPort->indRX = offsetof(struct mspResonse_s, payload);
    currentPort->dataSize = serialBuffer.mspResponse.header.size;
    const pgRegistry_t *regSet = pgMatcher(pgMatcherForMSPSet, (void*)&currentPort->cmdMSP);
    EXPECT_NE(static_cast<const pgRegistry_t*>(0), regSet);
    EXPECT_EQ(reg->address, regSet->address);

    mspProcessReceivedCommand();

    // check the values are as expected
    EXPECT_FLOAT_EQ(testBoardAlignment.rollDegrees, boardAlignment()->rollDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.pitchDegrees, boardAlignment()->pitchDegrees);
    EXPECT_FLOAT_EQ(testBoardAlignment.yawDegrees, boardAlignment()->yawDegrees);
}

TEST_F(SerialMspUnitTest, TestMspOutMessageLengthsCommand)
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
    for (uint ii = 0; ii < sizeof(outMessages); ++ii) {

        serialWritePos = 0;
        serialReadPos = 0;
        currentPort->cmdMSP = outMessages[ii];

#ifdef DEBUG_MSP
        printf("parse iteration: %d, MSP message id: %d\n", ii, currentPort->cmdMSP);
#endif

        mspProcessReceivedCommand();
        EXPECT_EQ('$', serialBuffer.mspResponse.header.dollar);
        EXPECT_EQ('M', serialBuffer.mspResponse.header.m);
        EXPECT_EQ('>', serialBuffer.mspResponse.header.direction);
        EXPECT_EQ(outMessages[ii], serialBuffer.mspResponse.header.type);
        // serial buffer includes mspHeader, payload and 1 byte checksum
        EXPECT_EQ(serialWritePos - sizeof(mspHeader_t) - 1, serialBuffer.mspResponse.header.size);
    }
}

// STUBS
extern "C" {
// from acceleration.c
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.
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

}

