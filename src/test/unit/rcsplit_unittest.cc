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


#include "gtest/gtest.h"

extern "C" {
    #include <stdbool.h>
    #include <stdint.h>
    #include <ctype.h>

    #include "platform.h"

    #include "common/utils.h"
    #include "common/maths.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"

    #include "io/beeper.h"
    #include "io/serial.h"

    #include "scheduler/scheduler.h"
    #include "drivers/serial.h"
    #include "io/rcsplit.h"

    #include "rx/rx.h"

    int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

    rcsplit_state_e unitTestRCsplitState()
    {
        return cameraState;
    }

    bool unitTestIsSwitchActivited(boxId_e boxId)
    {
        uint8_t adjustBoxID = boxId - BOXCAMERA1;
        rcsplit_switch_state_t switchState = switchStates[adjustBoxID];
        return switchState.isActivated;
    }

    void unitTestResetRCSplit()
    {
        rcSplitSerialPort = NULL;
        cameraState = RCSPLIT_STATE_UNKNOWN;
    }
}

typedef struct testData_s {
    bool isRunCamSplitPortConfigurated;
    bool isRunCamSplitOpenPortSupported;
    int8_t maxTimesOfRespDataAvailable;
    bool isAllowBufferReadWrite;
} testData_t;

static testData_t testData;

TEST(RCSplitTest, TestRCSplitInitWithoutPortConfigurated)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    bool result = rcSplitInit();
    EXPECT_EQ(false, result);
    EXPECT_EQ(RCSPLIT_STATE_UNKNOWN, unitTestRCsplitState());
}

TEST(RCSplitTest, TestRCSplitInitWithoutOpenPortConfigurated)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = false;
    testData.isRunCamSplitPortConfigurated = true;

    bool result = rcSplitInit();
    EXPECT_EQ(false, result);
    EXPECT_EQ(RCSPLIT_STATE_UNKNOWN, unitTestRCsplitState());
}

TEST(RCSplitTest, TestRCSplitInit)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;

    bool result = rcSplitInit();
    EXPECT_EQ(true, result);
    EXPECT_EQ(RCSPLIT_STATE_IS_READY, unitTestRCsplitState());
}

TEST(RCSplitTest, TestRecvWhoAreYouResponse)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    
    bool result = rcSplitInit();
    EXPECT_EQ(true, result);

    // here will generate a number in [6-255], it's make the serialRxBytesWaiting() and serialRead() run at least 5 times, 
    // so the "who are you response" will full received, and cause the state change to RCSPLIT_STATE_IS_READY;
    int8_t randNum = rand() % 127 + 6; 
    testData.maxTimesOfRespDataAvailable = randNum;
    rcSplitProcess((timeUs_t)0);

    EXPECT_EQ(RCSPLIT_STATE_IS_READY, unitTestRCsplitState());
}

TEST(RCSplitTest, TestWifiModeChangeWithDeviceUnready)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.maxTimesOfRespDataAvailable = 0;
    
    bool result = rcSplitInit();
    EXPECT_EQ(true, result);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= (BOXCAMERA3 - BOXCAMERA1); i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }

    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1800;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 900;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 900;

    updateUsedModeActivationConditionFlags();
    updateActivatedModes();

    // runn process loop
    rcSplitProcess(0);

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCSplitTest, TestWifiModeChangeWithDeviceReady)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.maxTimesOfRespDataAvailable = 0;

    bool result = rcSplitInit();
    EXPECT_EQ(true, result);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= BOXCAMERA3 - BOXCAMERA1; i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }


    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2000;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;

    updateUsedModeActivationConditionFlags();
    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6; 
    testData.maxTimesOfRespDataAvailable = randNum;
    rcSplitProcess((timeUs_t)0);

    EXPECT_EQ(RCSPLIT_STATE_IS_READY, unitTestRCsplitState());

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCSplitTest, TestWifiModeChangeCombine)
{
    memset(&testData, 0, sizeof(testData));
    unitTestResetRCSplit();
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.maxTimesOfRespDataAvailable = 0;
    
    bool result = rcSplitInit();
    EXPECT_EQ(true, result);

    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    for (uint8_t i = 0; i <= BOXCAMERA3 - BOXCAMERA1; i++) {
        memset(modeActivationConditionsMutable(i), 0, sizeof(modeActivationCondition_t));
    }


    // bind aux1 to wifi button with range [900,1600]
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->modeId = BOXCAMERA1;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(CHANNEL_RANGE_MIN);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(1600);

    // bind aux2 to power button with range [1900, 2100]
    modeActivationConditionsMutable(1)->auxChannelIndex = 1;
    modeActivationConditionsMutable(1)->modeId = BOXCAMERA2;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // bind aux3 to change mode with range [1300, 1600]
    modeActivationConditionsMutable(2)->auxChannelIndex = 2;
    modeActivationConditionsMutable(2)->modeId = BOXCAMERA3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    // // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2000;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1700;

    updateUsedModeActivationConditionFlags();
    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6; 
    testData.maxTimesOfRespDataAvailable = randNum;
    rcSplitProcess((timeUs_t)0);

    EXPECT_EQ(RCSPLIT_STATE_IS_READY, unitTestRCsplitState());

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));


    // // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1500;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1300;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1900;
    updateUsedModeActivationConditionFlags();
    updateActivatedModes();
    rcSplitProcess((timeUs_t)0);
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA3));


    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1899;
    updateUsedModeActivationConditionFlags();
    updateActivatedModes();
    rcSplitProcess((timeUs_t)0);
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));

    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2001;
    updateUsedModeActivationConditionFlags();
    updateActivatedModes();
    rcSplitProcess((timeUs_t)0);
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

extern "C" {
    serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, uint32_t baudRate, portMode_t mode, portOptions_t options)
    {
        UNUSED(identifier);
        UNUSED(functionMask);
        UNUSED(baudRate);
        UNUSED(callback);
        UNUSED(mode);
        UNUSED(options);

        if (testData.isRunCamSplitOpenPortSupported) {
            static serialPort_t s;
            s.vTable = NULL;

            // common serial initialisation code should move to serialPort::init()
            s.rxBufferHead = s.rxBufferTail = 0;
            s.txBufferHead = s.txBufferTail = 0;
            s.rxBufferSize = 0;
            s.txBufferSize = 0;
            s.rxBuffer = s.rxBuffer;
            s.txBuffer = s.txBuffer;

            // callback works for IRQ-based RX ONLY
            s.rxCallback = NULL;
            s.baudRate = 0;

            return (serialPort_t *)&s;
        }

        return NULL;
    }

    serialPortConfig_t *findSerialPortConfig(serialPortFunction_e function)
    {
        UNUSED(function);
        if (testData.isRunCamSplitPortConfigurated) {
            static serialPortConfig_t portConfig;

            portConfig.identifier = SERIAL_PORT_USART3;
            portConfig.msp_baudrateIndex = BAUD_115200;
            portConfig.gps_baudrateIndex = BAUD_57600;
            portConfig.telemetry_baudrateIndex = BAUD_AUTO;
            portConfig.peripheral_baudrateIndex = BAUD_115200;
            portConfig.functionMask = FUNCTION_MSP;

            return &portConfig;
        }

        return NULL;
    }

    uint32_t serialRxBytesWaiting(const serialPort_t *instance) 
    { 
        UNUSED(instance);

        testData.maxTimesOfRespDataAvailable--;
        if (testData.maxTimesOfRespDataAvailable > 0) {
            return 1;
        }

        return 0;
    }

    uint8_t serialRead(serialPort_t *instance) 
    { 
        UNUSED(instance); 

        if (testData.maxTimesOfRespDataAvailable > 0) {
            static uint8_t i = 0;
            static uint8_t buffer[] = { 0x55, 0x01, 0xFF, 0xad, 0xaa };

            if (i >= 5) {
                i = 0;
            }

            return buffer[i++];
        }

        return 0; 
    }

    void sbufWriteString(sbuf_t *dst, const char *string) 
    { 
        UNUSED(dst); UNUSED(string); 

        if (testData.isAllowBufferReadWrite) {
            sbufWriteData(dst, string, strlen(string));
        }
    }
    void sbufWriteU8(sbuf_t *dst, uint8_t val) 
    { 
        UNUSED(dst); UNUSED(val); 

        if (testData.isAllowBufferReadWrite) {
            *dst->ptr++ = val;
        }
    }
    
    void sbufWriteData(sbuf_t *dst, const void *data, int len)
    {
        UNUSED(dst); UNUSED(data); UNUSED(len); 

        if (testData.isAllowBufferReadWrite) {
            memcpy(dst->ptr, data, len);
            dst->ptr += len;
            
        }
    }

    // modifies streambuf so that written data are prepared for reading
    void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
    {
        UNUSED(buf); UNUSED(base); 

        if (testData.isAllowBufferReadWrite) {
            buf->end = buf->ptr;
            buf->ptr = base;
        }
    }

    bool feature(uint32_t) { return false;}
    void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) { UNUSED(instance); UNUSED(data); UNUSED(count); }

    void accSetCalibrationCycles(uint16_t calibrationCyclesRequired) { UNUSED(calibrationCyclesRequired); }
    void applyAndSaveBoardAlignmentDelta(int16_t roll, int16_t pitch) { UNUSED(roll); UNUSED(pitch); }

    uint16_t armingFlags = 0;
    void beeper(beeperMode_e) {}
    bool failsafeIsActive(void) { return false; }
    void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired) { UNUSED(calibrationCyclesRequired); };
    timeMs_t millis(void) { return 0; }
    void mwArm(void) {}
    void mwDisarm(void) {}

    void saveConfigAndNotify(void) {}
    void setConfigProfileAndWriteEEPROM(uint8_t profileIndex) { UNUSED(profileIndex); }
    uint8_t stateFlags;

    void failsafeOnRxResume(void) {}
    void failsafeOnRxSuspend(uint32_t usSuspendPeriod) { UNUSED(usSuspendPeriod); }
    void failsafeOnValidDataFailed(void) { }
    void failsafeOnValidDataReceived(void) { }
    uint32_t micros(void) { return 0; }
}
