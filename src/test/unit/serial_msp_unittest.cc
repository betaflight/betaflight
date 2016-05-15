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

extern "C" {
    #include <platform.h>
    #include "build_config.h"
    #include "version.h"
    #include "debug.h"
    #include "common/axis.h"
    #include "common/color.h"
    #include "common/maths.h"

    #include "drivers/system.h"
    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"
    #include "drivers/compass.h"
    #include "drivers/serial.h"
    #include "drivers/bus_i2c.h"
    #include "drivers/gpio.h"
    #include "drivers/timer.h"
    #include "drivers/pwm_rx.h"
    #include "drivers/buf_writer.h"

    #include "rx/rx.h"

    #include "io/rc_controls.h"
    #include "io/gps.h"
    #include "io/gimbal.h"
    #include "io/ledstrip.h"
    #include "io/serial_msp.h"
    #include "io/escservo.h"

    #include "telemetry/telemetry.h"

    #include "sensors/sensors.h"
    #include "sensors/boardalignment.h"
    #include "sensors/sensors.h"
    #include "sensors/battery.h"
    #include "sensors/sonar.h"
    #include "sensors/acceleration.h"
    #include "sensors/barometer.h"
    #include "sensors/compass.h"
    #include "sensors/gyro.h"

    #include "flight/mixer.h"
    #include "flight/pid.h"
    #include "flight/navigation.h"
    #include "flight/imu.h"
    #include "flight/failsafe.h"

    #include "config/runtime_config.h"
    #include "config/config.h"
    #include "config/config_profile.h"
    #include "config/config_master.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"


extern "C" {
    void setCurrentPort(mspPort_t *port);
    void mspProcessReceivedCommand();
    extern mspPort_t *currentPort;
    extern bufWriter_t *writer;
    extern mspPort_t mspPorts[];
    profile_t *currentProfile;
}

profile_t profile;

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
    currentProfile = &profile;
    currentProfile->pidProfile.pidController = PID_CONTROLLER_MWREWRITE;
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
    currentProfile = &profile;
    currentProfile->pidProfile.pidController = PID_CONTROLLER_MWREWRITE;
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_PID_CONTROLLER;
    mspProcessReceivedCommand();
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, serialBuffer.mspResponse.payload[0]);

    // set the pidController to a different value so we can check if it gets read back properly
    currentProfile->pidProfile.pidController = PID_CONTROLLER_LUX_FLOAT;

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
    EXPECT_EQ(PID_CONTROLLER_MWREWRITE, currentProfile->pidProfile.pidController);
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

    currentProfile = &profile;
    currentProfile->pidProfile.pidController = PID_CONTROLLER_MWREWRITE;
    currentProfile->pidProfile.P8[PIDROLL] = P8_ROLL;
    currentProfile->pidProfile.I8[PIDROLL] = I8_ROLL;
    currentProfile->pidProfile.D8[PIDROLL] = D8_ROLL;
    currentProfile->pidProfile.P8[PIDPITCH] = P8_PITCH;
    currentProfile->pidProfile.I8[PIDPITCH] = I8_PITCH;
    currentProfile->pidProfile.D8[PIDPITCH] = D8_PITCH;
    currentProfile->pidProfile.P8[PIDYAW] = P8_YAW;
    currentProfile->pidProfile.I8[PIDYAW] = I8_YAW;
    currentProfile->pidProfile.D8[PIDYAW] = D8_YAW;
    currentProfile->pidProfile.P8[PIDALT] = P8_PIDALT;
    currentProfile->pidProfile.I8[PIDALT] = I8_PIDALT;
    currentProfile->pidProfile.D8[PIDALT] = D8_PIDALT;
    currentProfile->pidProfile.P8[PIDPOS] = P8_PIDPOS;
    currentProfile->pidProfile.I8[PIDPOS] = I8_PIDPOS;
    currentProfile->pidProfile.D8[PIDPOS] = D8_PIDPOS;
    currentProfile->pidProfile.P8[PIDPOSR] = P8_PIDPOSR;
    currentProfile->pidProfile.I8[PIDPOSR] = I8_PIDPOSR;
    currentProfile->pidProfile.D8[PIDPOSR] = D8_PIDPOSR;
    currentProfile->pidProfile.P8[PIDNAVR] = P8_PIDNAVR;
    currentProfile->pidProfile.I8[PIDNAVR] = I8_PIDNAVR;
    currentProfile->pidProfile.D8[PIDNAVR] = D8_PIDNAVR;
    currentProfile->pidProfile.P8[PIDLEVEL] = P8_PIDLEVEL;
    currentProfile->pidProfile.I8[PIDLEVEL] = I8_PIDLEVEL;
    currentProfile->pidProfile.D8[PIDLEVEL] = D8_PIDLEVEL;
    currentProfile->pidProfile.P8[PIDMAG] = P8_PIDMAG;
    currentProfile->pidProfile.P8[PIDVEL] = P8_PIDVEL;
    currentProfile->pidProfile.I8[PIDVEL] = I8_PIDVEL;
    currentProfile->pidProfile.D8[PIDVEL] = D8_PIDVEL;

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
    memset(&currentProfile->pidProfile, 0, sizeof(currentProfile->pidProfile));

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
    EXPECT_EQ(P8_ROLL, currentProfile->pidProfile.P8[PIDROLL]);
    EXPECT_EQ(I8_ROLL, currentProfile->pidProfile.I8[PIDROLL]);
    EXPECT_EQ(D8_ROLL, currentProfile->pidProfile.D8[PIDROLL]);
    EXPECT_EQ(P8_PITCH, currentProfile->pidProfile.P8[PIDPITCH]);
    EXPECT_EQ(I8_PITCH, currentProfile->pidProfile.I8[PIDPITCH]);
    EXPECT_EQ(D8_PITCH, currentProfile->pidProfile.D8[PIDPITCH]);
    EXPECT_EQ(P8_YAW, currentProfile->pidProfile.P8[PIDYAW]);
    EXPECT_EQ(I8_YAW, currentProfile->pidProfile.I8[PIDYAW]);
    EXPECT_EQ(D8_YAW, currentProfile->pidProfile.D8[PIDYAW]);
    EXPECT_EQ(P8_PIDALT, currentProfile->pidProfile.P8[PIDALT]);
    EXPECT_EQ(I8_PIDALT, currentProfile->pidProfile.I8[PIDALT]);
    EXPECT_EQ(D8_PIDALT, currentProfile->pidProfile.D8[PIDALT]);
    EXPECT_EQ(P8_PIDPOS, currentProfile->pidProfile.P8[PIDPOS]);
    EXPECT_EQ(I8_PIDPOS, currentProfile->pidProfile.I8[PIDPOS]);
    EXPECT_EQ(D8_PIDPOS, currentProfile->pidProfile.D8[PIDPOS]);
    EXPECT_EQ(P8_PIDPOSR, currentProfile->pidProfile.P8[PIDPOSR]);
    EXPECT_EQ(I8_PIDPOSR, currentProfile->pidProfile.I8[PIDPOSR]);
    EXPECT_EQ(D8_PIDPOSR, currentProfile->pidProfile.D8[PIDPOSR]);
    EXPECT_EQ(P8_PIDNAVR, currentProfile->pidProfile.P8[PIDNAVR]);
    EXPECT_EQ(I8_PIDNAVR, currentProfile->pidProfile.I8[PIDNAVR]);
    EXPECT_EQ(D8_PIDNAVR, currentProfile->pidProfile.D8[PIDNAVR]);
    EXPECT_EQ(P8_PIDLEVEL, currentProfile->pidProfile.P8[PIDLEVEL]);
    EXPECT_EQ(I8_PIDLEVEL, currentProfile->pidProfile.I8[PIDLEVEL]);
    EXPECT_EQ(D8_PIDLEVEL, currentProfile->pidProfile.D8[PIDLEVEL]);
    EXPECT_EQ(P8_PIDMAG, currentProfile->pidProfile.P8[PIDMAG]);
    EXPECT_EQ(P8_PIDVEL, currentProfile->pidProfile.P8[PIDVEL]);
    EXPECT_EQ(I8_PIDVEL, currentProfile->pidProfile.I8[PIDVEL]);
    EXPECT_EQ(D8_PIDVEL, currentProfile->pidProfile.D8[PIDVEL]);
}

TEST_F(SerialMspUnitTest, Test_PIDValuesFloat)
{
    // check the buffer is big enough for the data to read in
    EXPECT_LE(sizeof(mspHeader_t) + 3 * PID_ITEM_COUNT + 1, MSP_PORT_INBUF_SIZE); // +1 for checksum

    // set up some test data
    // use test values close to default, but make sure they are all different
    const float Pf_ROLL = 1.4f;
    const float If_ROLL = 0.4f;
    const float Df_ROLL = 0.03f;
    const float Pf_PITCH = 1.5f; // default 1.4
    const float If_PITCH = 0.5f; // default 0.4
    const float Df_PITCH = 0.02f; // default 0.03
    const float Pf_YAW = 3.5f;
    const float If_YAW = 0.6f; // default 0.4
    const float Df_YAW = 0.01;
    const float A_level = 5.0f;
    const float H_level = 3.0f;
    const uint8_t H_sensitivity = 75;
    currentProfile = &profile;
    currentProfile->pidProfile.pidController = PID_CONTROLLER_LUX_FLOAT;
    currentProfile->pidProfile.P_f[PIDROLL] = Pf_ROLL;
    currentProfile->pidProfile.I_f[PIDROLL] = If_ROLL;
    currentProfile->pidProfile.D_f[PIDROLL] = Df_ROLL;
    currentProfile->pidProfile.P_f[PIDPITCH] = Pf_PITCH;
    currentProfile->pidProfile.I_f[PIDPITCH] = If_PITCH;
    currentProfile->pidProfile.D_f[PIDPITCH] = Df_PITCH;
    currentProfile->pidProfile.P_f[PIDYAW] = Pf_YAW;
    currentProfile->pidProfile.I_f[PIDYAW] = If_YAW;
    currentProfile->pidProfile.D_f[PIDYAW] = Df_YAW;
    currentProfile->pidProfile.A_level = A_level;
    currentProfile->pidProfile.H_level = H_level;
    currentProfile->pidProfile.H_sensitivity = H_sensitivity;

    // use the MSP to write out the PID values
    serialWritePos = 0;
    serialReadPos = 0;
    currentPort->cmdMSP = MSP_PID;
    mspProcessReceivedCommand();
    EXPECT_EQ(3 * PID_ITEM_COUNT, serialBuffer.mspResponse.header.size);

    // reset test values to zero, so we can check if they get read properly
    memset(&currentProfile->pidProfile, 0, sizeof(currentProfile->pidProfile));

    // now use the MSP to read back the PID values and check they are the same
    // spoof a change from the written MSP_PID to the readable MSP_SET_PID
    currentPort->cmdMSP = MSP_SET_PID;
    serialBuffer.mspResponse.header.direction = '<';
    serialBuffer.mspResponse.header.type = currentPort->cmdMSP;
    // force the checksum
    serialBuffer.mspResponse.payload[3 * PID_ITEM_COUNT] ^= MSP_PID;
    serialBuffer.mspResponse.payload[3 * PID_ITEM_COUNT] ^= MSP_SET_PID;
    // copy the command data into the current port inBuf so it can be processed
    memcpy(currentPort->inBuf, serialBuffer.buf, MSP_PORT_INBUF_SIZE);

    // need to reset the controller for values to be read back correctly
    currentProfile->pidProfile.pidController = PID_CONTROLLER_LUX_FLOAT;

    // set the offset into the payload
    currentPort->indRX = offsetof(struct mspResonse_s, payload);
    mspProcessReceivedCommand();

    // check the values are as expected
    EXPECT_FLOAT_EQ(Pf_ROLL, currentProfile->pidProfile.P_f[PIDROLL]);
    EXPECT_FLOAT_EQ(If_ROLL, currentProfile->pidProfile.I_f[PIDROLL]);
    EXPECT_FLOAT_EQ(Df_ROLL, currentProfile->pidProfile.D_f[PIDROLL]);
    EXPECT_FLOAT_EQ(Pf_PITCH, currentProfile->pidProfile.P_f[PIDPITCH]);
    EXPECT_FLOAT_EQ(If_PITCH, currentProfile->pidProfile.I_f[PIDPITCH]);
    EXPECT_FLOAT_EQ(Df_PITCH, currentProfile->pidProfile.D_f[PIDPITCH]);
    EXPECT_FLOAT_EQ(Pf_YAW, currentProfile->pidProfile.P_f[PIDYAW]);
    EXPECT_FLOAT_EQ(If_YAW, currentProfile->pidProfile.I_f[PIDYAW]);
    EXPECT_FLOAT_EQ(Df_YAW, currentProfile->pidProfile.D_f[PIDYAW]);
    EXPECT_FLOAT_EQ(A_level, currentProfile->pidProfile.A_level);
    EXPECT_FLOAT_EQ(H_level, currentProfile->pidProfile.H_level);
    EXPECT_EQ(H_sensitivity, currentProfile->pidProfile.H_sensitivity);
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
master_t masterConfig;
controlRateConfig_t *currentControlRateProfile;
void resetPidProfile(pidProfile_t *pidProfile) {UNUSED(pidProfile);}
void handleOneshotFeatureChangeOnRestart(void) {}
void readEEPROM(void) {}
void resetEEPROM(void) {}
void writeEEPROM(void) {}
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
void rxMspFrameReceive(uint16_t *frame, int channelCount) {UNUSED(frame);UNUSED(channelCount);}
// from mw.c
uint16_t cycleTime = 0;         // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
// from navigation.c
int32_t GPS_home[2];
int32_t GPS_hold[2];
uint16_t GPS_distanceToHome;        // distance to home point in meters
int16_t GPS_directionToHome;        // direction to home or hol point in degrees
navigationMode_e nav_mode = NAV_MODE_NONE;    // Navigation mode
void GPS_set_next_wp(int32_t *lat, int32_t *lon) {UNUSED(lat);UNUSED(lon);}
// from pid.c
void pidSetController(pidControllerType_e type) {UNUSED(type);}
// from rc_controls.c
uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e
void useRcControlsConfig(void *modeActivationConditions, void *escAndServoConfigToUse, void *pidProfileToUse) {
    UNUSED(modeActivationConditions);UNUSED(escAndServoConfigToUse);UNUSED(pidProfileToUse);}
// from runtime_config.c
uint8_t armingFlags = 0;
uint8_t stateFlags = 0;
uint16_t flightModeFlags = 0;
uint16_t disableFlightMode(flightModeFlags_e mask) {UNUSED(mask);return 0;}
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
}
