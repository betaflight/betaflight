/*
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

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "blackbox/blackbox.h"
    #include "common/utils.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"
    #include "pg/motor.h"

    #include "drivers/accgyro/accgyro.h"
    #include "drivers/accgyro/gyro_sync.h"
    #include "drivers/serial.h"

    #include "flight/failsafe.h"
    #include "flight/mixer.h"
    #include "flight/pid.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"

    #include "io/gps.h"
    #include "io/serial.h"

    #include "rx/rx.h"

    #include "sensors/battery.h"
    #include "sensors/gyro.h"

    extern int16_t blackboxIInterval;
    extern int16_t blackboxPInterval;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

gyroDev_t gyroDev;

TEST(BlackboxTest, TestInitIntervals)
{
    blackboxConfigMutable()->sample_rate = 4; // sample_rate = PID loop frequency / 16
    // 250Hz PIDloop
    targetPidLooptime = 4000;
    blackboxInit();
    EXPECT_EQ(8, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_EQ(2048, blackboxSInterval);

    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_EQ(16, blackboxIInterval);
    EXPECT_EQ(16, blackboxPInterval);
    EXPECT_EQ(4096, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 1; // sample_rate = PID loop frequency / 2
    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(16384, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 2;
    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(32768, blackboxSInterval);

    blackboxConfigMutable()->sample_rate = 3;
    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(65536, blackboxSInterval);
}

TEST(BlackboxTest, Test_500Hz)
{
    blackboxConfigMutable()->sample_rate = 0;
    // 500Hz PIDloop
    targetPidLooptime = 2000;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    for (int ii = 0; ii < 15; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_1kHz)
{
    blackboxConfigMutable()->sample_rate = 0;
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_FALSE(blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_TRUE(writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_FALSE(writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_2kHz)
{
    blackboxConfigMutable()->sample_rate = 1;
    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    blackboxAdvanceIterationTimers();
    EXPECT_FALSE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());

    for (int ii = 0; ii < 31; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_TRUE(blackboxShouldLogPFrame());

        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_FALSE(blackboxShouldLogPFrame());
    }

    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());

    blackboxAdvanceIterationTimers();
    EXPECT_FALSE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_8kHz)
{
    blackboxConfigMutable()->sample_rate = 3;
    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);
    EXPECT_FALSE(blackboxSlowFrameIterationTimer >= blackboxSInterval);
    blackboxSlowFrameIterationTimer = blackboxSInterval;
    EXPECT_TRUE(writeSlowFrameIfNeeded());
    EXPECT_EQ(0, blackboxSlowFrameIterationTimer);

    for (int ii = 0; ii < 255; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_EQ(ii + 1, blackboxSlowFrameIterationTimer);
        EXPECT_FALSE(writeSlowFrameIfNeeded());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_TRUE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_zero_p_interval)
{
    blackboxConfigMutable()->sample_rate = 4;
    // 250Hz PIDloop
    targetPidLooptime = 4000;
    blackboxInit();
    EXPECT_EQ(8, blackboxIInterval);
    EXPECT_EQ(0, blackboxPInterval);
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());

    for (int ii = 0; ii < 7; ++ii) {
        blackboxAdvanceIterationTimers();
        EXPECT_FALSE(blackboxShouldLogIFrame());
        EXPECT_FALSE(blackboxShouldLogPFrame());
    }
    blackboxAdvanceIterationTimers();
    EXPECT_TRUE(blackboxShouldLogIFrame());
    EXPECT_FALSE(blackboxShouldLogPFrame());
}

TEST(BlackboxTest, Test_CalculatePDenom)
{
    blackboxConfigMutable()->sample_rate = 0;
    // note I-frame is logged every 32ms regardless of PIDloop rate

    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 1)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(8, blackboxCalculatePDenom(1, 4));

    // 2kHz PIDloop
    targetPidLooptime = 500;
    blackboxInit();
    EXPECT_EQ(64, blackboxIInterval);
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 2)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 4));

    // 4kHz PIDloop
    targetPidLooptime = 250;
    blackboxInit();
    EXPECT_EQ(128, blackboxIInterval);
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 4)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 8));

    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(256, blackboxCalculatePDenom(1, 1));
    EXPECT_EQ(128, blackboxCalculatePDenom(1, 2));
    EXPECT_EQ(64, blackboxCalculatePDenom(1, 4));
    EXPECT_EQ(32, blackboxCalculatePDenom(1, 8)); // 1kHz logging
    EXPECT_EQ(16, blackboxCalculatePDenom(1, 16));
}

TEST(BlackboxTest, Test_CalculateRates)
{
    // 1kHz PIDloop
    targetPidLooptime = 1000;
    blackboxConfigMutable()->sample_rate = 0;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 1;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 2;
    blackboxInit();
    EXPECT_EQ(32, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());


    // 8kHz PIDloop
    targetPidLooptime = 125;
    blackboxConfigMutable()->sample_rate = 3; // 1kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(8, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 4; // 500Hz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(16, blackboxPInterval);
    EXPECT_EQ(16, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 2; // 2kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(4, blackboxPInterval);
    EXPECT_EQ(4, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 1; // 4kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(2, blackboxPInterval);
    EXPECT_EQ(2, blackboxGetRateDenom());

    blackboxConfigMutable()->sample_rate = 0; // 8kHz logging
    blackboxInit();
    EXPECT_EQ(256, blackboxIInterval);
    EXPECT_EQ(1, blackboxPInterval);
    EXPECT_EQ(1, blackboxGetRateDenom());

    // 0.126 PIDloop = 7.94kHz
    targetPidLooptime = 126;
    blackboxConfigMutable()->sample_rate = 3; // 0.992kHz logging
    blackboxInit();
    EXPECT_EQ(253, blackboxIInterval);
    EXPECT_EQ(8, blackboxPInterval);
    EXPECT_EQ(8, blackboxGetRateDenom());

}


// STUBS
extern "C" {

PG_REGISTER(flight3DConfig_t, flight3DConfig, PG_MOTOR_3D_CONFIG, 0);
PG_REGISTER(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 0);
PG_REGISTER(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 0);
PG_REGISTER(batteryConfig_t, batteryConfig, PG_BATTERY_CONFIG, 0);
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
PG_REGISTER_ARRAY(modeActivationCondition_t, MAX_MODE_ACTIVATION_CONDITION_COUNT, modeActivationConditions, PG_MODE_ACTIVATION_PROFILE, 0);

uint8_t armingFlags;
uint8_t stateFlags;
const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e
uint8_t debugMode = 0;
int16_t debug[DEBUG16_VALUE_COUNT];
int32_t blackboxHeaderBudget;
gpsSolutionData_t gpsSol;
int32_t GPS_home[2];

gyro_t gyro;

float motor_disarmed[MAX_SUPPORTED_MOTORS];
struct pidProfile_s;
struct pidProfile_s *currentPidProfile;
uint32_t targetPidLooptime;

boxBitmask_t rcModeActivationMask;

void mspSerialAllocatePorts(void) {}
uint32_t getArmingBeepTimeMicros(void) {return 0;}
uint16_t getBatteryVoltageLatest(void) {return 0;}
uint8_t getMotorCount(void) {return 4;}
bool areMotorsRunning(void) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) {return false;}
bool isModeActivationConditionPresent(boxId_e) {return false;}
uint32_t millis(void) {return 0;}
bool sensors(uint32_t) {return false;}
void serialWrite(serialPort_t *, uint8_t) {}
uint32_t serialTxBytesFree(const serialPort_t *) {return 0;}
bool isSerialTransmitBufferEmpty(const serialPort_t *) {return false;}
bool featureIsEnabled(uint32_t) {return false;}
void mspSerialReleasePortIfAllocated(serialPort_t *) {}
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e ) {return NULL;}
serialPort_t *findSharedSerialPort(uint16_t , serialPortFunction_e ) {return NULL;}
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {return NULL;}
void closeSerialPort(serialPort_t *) {}
portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e ) {return PORTSHARING_UNUSED;}
failsafePhase_e failsafePhase(void) {return FAILSAFE_IDLE;}
bool rxAreFlightChannelsValid(void) {return false;}
bool rxIsReceivingSignal(void) {return false;}
bool isRssiConfigured(void) {return false;}
float getMotorOutputLow(void) {return 0.0;}
float getMotorOutputHigh(void) {return 0.0;}
}
