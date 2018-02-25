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

    #include "common/bitarray.h"
    #include "common/maths.h"
    #include "common/utils.h"
    #include "common/streambuf.h"

    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"

    #include "drivers/serial.h"

    #include "io/beeper.h"
    #include "io/serial.h"

    #include "scheduler/scheduler.h"
    #include "io/rcdevice_cam.h"
    #include "io/osd.h"
    #include "io/rcdevice.h"
    #include "io/rcdevice_osd.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/vcd.h"

    #include "rx/rx.h"

    int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]

    extern rcdeviceSwitchState_t switchStates[BOXCAMERA3 - BOXCAMERA1 + 1];
    extern runcamDevice_t *camDevice;
    extern bool needRelease;
    bool unitTestIsSwitchActivited(boxId_e boxId)
    {
        uint8_t adjustBoxID = boxId - BOXCAMERA1;
        rcdeviceSwitchState_s switchState = switchStates[adjustBoxID];
        return switchState.isActivated;
    }
}

#define MAX_RESPONSES_COUNT 10
#define FIVE_KEY_JOYSTICK_MIN FIVE_KEY_CABLE_JOYSTICK_MIN - 1
#define FIVE_KEY_JOYSTICK_MID FIVE_KEY_CABLE_JOYSTICK_MID_START + 1
#define FIVE_KEY_JOYSTICK_MAX FIVE_KEY_CABLE_JOYSTICK_MAX + 1

typedef struct testData_s {
    bool isRunCamSplitPortConfigurated;
    bool isRunCamSplitOpenPortSupported;
    int8_t maxTimesOfRespDataAvailable;
    bool isAllowBufferReadWrite;
    uint8_t indexOfCurrentRespBuf;
    uint8_t responseBufCount;
    uint8_t responesBufs[MAX_RESPONSES_COUNT][RCDEVICE_PROTOCOL_MAX_PACKET_SIZE];
    uint8_t responseBufsLen[MAX_RESPONSES_COUNT];
    uint8_t responseDataReadPos;
    uint32_t millis;
} testData_t;

static testData_t testData;

static void clearResponseBuff()
{
    testData.indexOfCurrentRespBuf = 0;
    testData.responseBufCount = 0;
    memset(testData.responseBufsLen, 0, MAX_RESPONSES_COUNT);
    memset(testData.responesBufs, 0, MAX_RESPONSES_COUNT * 60);
}

static void addResponseData(uint8_t *data, uint8_t dataLen, bool withDataForFlushSerial)
{
    if (withDataForFlushSerial) {
        memcpy(testData.responesBufs[testData.responseBufCount], "0", 1);
        testData.responseBufsLen[testData.responseBufCount] = 1;
        testData.responseBufCount++;
    }

    
    memcpy(testData.responesBufs[testData.responseBufCount], data, dataLen);
    testData.responseBufsLen[testData.responseBufCount] = dataLen;
    testData.responseBufCount++;
}

TEST(RCDeviceTest, TestRCSplitInitWithoutPortConfigurated)
{
    runcamDevice_t device;

    memset(&testData, 0, sizeof(testData));
    bool result = runcamDeviceInit(&device);
    EXPECT_EQ(false, result);
}

TEST(RCDeviceTest, TestRCSplitInitWithoutOpenPortConfigurated)
{
    runcamDevice_t device;

    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = false;
    testData.isRunCamSplitPortConfigurated = true;

    bool result = runcamDeviceInit(&device);
    EXPECT_EQ(false, result);
}

TEST(RCDeviceTest, TestInitDevice)
{
    runcamDevice_t device;

    // test correct response
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    
    bool result = runcamDeviceInit(&device);
    EXPECT_EQ(result, true);
}

TEST(RCDeviceTest, TestInitDeviceWithInvalidResponse)
{
    runcamDevice_t device;

    // test correct response data with incorrect len
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;

    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD, 0x33 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = runcamDeviceInit(&device);
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // invalid crc
    uint8_t responseDataWithInvalidCRC[] = { 0xCC, 0x01, 0x37, 0x00, 0xBE };
    addResponseData(responseDataWithInvalidCRC, sizeof(responseDataWithInvalidCRC), true);
    result = runcamDeviceInit(&device);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // incomplete response data
    uint8_t incompleteResponseData[] = { 0xCC, 0x01, 0x37 };
    addResponseData(incompleteResponseData, sizeof(incompleteResponseData), true);
    result = runcamDeviceInit(&device);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test timeout
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    result = runcamDeviceInit(&device);
    EXPECT_EQ(result, false);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestWifiModeChangeWithDeviceUnready)
{
    // test correct response
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBC };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, false);

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

    updateActivatedModes();

    // runn process loop
    rcdeviceUpdate(0);

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCDeviceTest, TestWifiModeChangeWithDeviceReady)
{
    // test correct response
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);

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

    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6;
    testData.maxTimesOfRespDataAvailable = randNum;
    rcdeviceUpdate((timeUs_t)0);

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCDeviceTest, TestWifiModeChangeCombine)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
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
    updateActivatedModes();

    // runn process loop
    int8_t randNum = rand() % 127 + 6;
    testData.maxTimesOfRespDataAvailable = randNum;
    rcdeviceUpdate((timeUs_t)0);

    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));


    // // make the binded mode inactive
    rcData[modeActivationConditions(0)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1500;
    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1300;
    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1900;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA3));


    rcData[modeActivationConditions(2)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 1899;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));

    rcData[modeActivationConditions(1)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = 2001;
    updateActivatedModes();
    rcdeviceUpdate((timeUs_t)0);
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA1));
    EXPECT_EQ(true, unitTestIsSwitchActivited(BOXCAMERA2));
    EXPECT_EQ(false, unitTestIsSwitchActivited(BOXCAMERA3));
}

TEST(RCDeviceTest, Test5KeyOSDCableSimulationProtocol)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(true, result);

    // test timeout of open connection
    result = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // open connection with correct response
    uint8_t responseDataOfOpenConnection[] = { 0xCC, 0x11, 0xe7 };
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    result = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // open connection with correct response but wrong data length 
    uint8_t incorrectResponseDataOfOpenConnection1[] = { 0xCC, 0x11, 0xe7, 0x55 };
    addResponseData(incorrectResponseDataOfOpenConnection1, sizeof(incorrectResponseDataOfOpenConnection1), true);
    result = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
    EXPECT_EQ(true, result);
    clearResponseBuff();
    
    // open connection with invalid crc
    uint8_t incorrectResponseDataOfOpenConnection2[] = { 0xCC, 0x10, 0x42 };
    addResponseData(incorrectResponseDataOfOpenConnection2, sizeof(incorrectResponseDataOfOpenConnection2), true);
    result = runcamDeviceOpen5KeyOSDCableConnection(camDevice);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // test timeout of close connection
    runcamDeviceClose5KeyOSDCableConnection(camDevice, NULL);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // close connection with correct response
    uint8_t responseDataOfCloseConnection[] = { 0xCC, 0x21, 0x11 };
    addResponseData(responseDataOfCloseConnection, sizeof(responseDataOfCloseConnection), true);
    result = runcamDeviceClose5KeyOSDCableConnection(camDevice, NULL);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // close connection with correct response but wrong data length 
    uint8_t responseDataOfCloseConnection1[] = { 0xCC, 0x21, 0x11, 0xC1 };
    addResponseData(responseDataOfCloseConnection1, sizeof(responseDataOfCloseConnection1), true);
    result = runcamDeviceClose5KeyOSDCableConnection(camDevice, NULL);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // close connection with response that invalid crc
    uint8_t responseDataOfCloseConnection2[] = { 0xCC, 0x21, 0xA1 };
    addResponseData(responseDataOfCloseConnection2, sizeof(responseDataOfCloseConnection2), true);
    result = runcamDeviceClose5KeyOSDCableConnection(camDevice, NULL);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // simulate press button with no response
    result = runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // simulate press button with correct response
    uint8_t responseDataOfSimulation1[] = { 0xCC, 0xA5 };
    addResponseData(responseDataOfSimulation1, sizeof(responseDataOfSimulation1), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // simulate press button with correct response but wrong data length 
    uint8_t responseDataOfSimulation2[] = { 0xCC, 0xA5, 0x22 };
    addResponseData(responseDataOfSimulation2, sizeof(responseDataOfSimulation2), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // simulate press button event with incorrect response
    uint8_t responseDataOfSimulation3[] = { 0xCC, 0xB5, 0x22 };
    addResponseData(responseDataOfSimulation3, sizeof(responseDataOfSimulation3), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // simulate release button event 
    result = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
    EXPECT_EQ(false, result);
    clearResponseBuff();

    // simulate release button with correct response
    uint8_t responseDataOfSimulation4[] = { 0xCC, 0xA5 };
    addResponseData(responseDataOfSimulation4, sizeof(responseDataOfSimulation4), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // simulate release button with correct response but wrong data length
    uint8_t responseDataOfSimulation5[] = { 0xCC, 0xA5, 0xFF };
    addResponseData(responseDataOfSimulation5, sizeof(responseDataOfSimulation5), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
    EXPECT_EQ(true, result);
    clearResponseBuff();

    // simulate release button with incorrect response
    uint8_t responseDataOfSimulation6[] = { 0xCC, 0x31, 0xFF };
    addResponseData(responseDataOfSimulation6, sizeof(responseDataOfSimulation6), true);
    result = runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice);
    EXPECT_EQ(false, result);
    clearResponseBuff();
}

TEST(RCDeviceTest, Test5KeyOSDCableSimulationWithout5KeyFeatureSupport)
{
    // test simulation without device init
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MAX; // Yaw High
    rcdeviceUpdate(0);
    EXPECT_EQ(false, rcdeviceInMenu);
    
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // open connection, rcdeviceInMenu will be false if the codes is right
    uint8_t responseDataOfOpenConnection[] = { 0xCC, 0x11, 0xe7 };
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), false);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, rcdeviceInMenu);
    clearResponseBuff();
}

TEST(RCDeviceTest, Test5KeyOSDCableSimulationWith5KeyFeatureSupport)
{
    // test simulation without device init
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MAX; // Yaw High
    rcdeviceUpdate(0);
    EXPECT_EQ(false, rcdeviceInMenu);

    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // open connection
    uint8_t responseDataOfOpenConnection[] = { 0xCC, 0x11, 0xe7 };
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, rcdeviceInMenu);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    uint8_t responseDataOfReleaseButton[] = { 0xCC, 0xA5 };
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    clearResponseBuff();

    // close connection
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MIN; // Yaw Low
    uint8_t responseDataOfCloseConnection[] = { 0xCC, 0x21, 0x11 };
    addResponseData(responseDataOfCloseConnection, sizeof(responseDataOfCloseConnection), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, rcdeviceInMenu);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    clearResponseBuff();

    // open osd menu again
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MAX; // Yaw High
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, rcdeviceInMenu);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    clearResponseBuff();

    // send down button event
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MIN;
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // rest down button
    clearResponseBuff();

    // simulate right button long press
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MAX;
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // send relase button event
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // reset right button
    clearResponseBuff();

    // simulate right button and get failed response, then FC should release the controller of joysticks
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MAX;
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // send relase button with empty response, so the release will failed
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    EXPECT_EQ(false, rcdeviceInMenu); // if failed on release button event, then FC side need release the controller of joysticks
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // rest right button
    // send again release button with correct response
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    clearResponseBuff();
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    EXPECT_EQ(false, rcdeviceInMenu);
    clearResponseBuff();

    // open OSD menu again
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MAX; // Yaw High
    clearResponseBuff();
    addResponseData(responseDataOfOpenConnection, sizeof(responseDataOfOpenConnection), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, rcdeviceInMenu);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();

    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    clearResponseBuff();

    // send left event
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MIN;
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // send relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    EXPECT_EQ(true, rcdeviceInMenu);
    clearResponseBuff();
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // rest right button

    // close connection
    rcData[THROTTLE] = FIVE_KEY_JOYSTICK_MID; // THROTTLE Mid
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MIN; // Yaw High
    addResponseData(responseDataOfCloseConnection, sizeof(responseDataOfCloseConnection), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, rcdeviceInMenu);
    EXPECT_EQ(true, needRelease);
    clearResponseBuff();
    // relase button 
    rcData[ROLL] = FIVE_KEY_JOYSTICK_MID; // ROLL Mid
    rcData[PITCH] = FIVE_KEY_JOYSTICK_MID; // PITCH Mid
    rcData[YAW] = FIVE_KEY_JOYSTICK_MID; // Yaw Mid
    addResponseData(responseDataOfReleaseButton, sizeof(responseDataOfReleaseButton), true);
    rcdeviceUpdate(0);
    EXPECT_EQ(false, needRelease);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestOSDDeviceSupportWithAutoDetectVideoSystem)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    uint8_t respDataOfGettingColumnsDetail[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0 };
    uint8_t respDataOfGettingTVModeDetail[] = { 0xCC, 0x00, 0x0B, 0x09, 0x00, 0x4E, 0x54, 0x53, 0x43, 0x3B, 0x50, 0x41, 0x4C, 0x00, 0x08 };
    uint8_t respDataOfWriteBFCharset[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responseData, sizeof(responseData), true); // device init response
    addResponseData(respDataOfGettingColumnsDetail, sizeof(respDataOfGettingColumnsDetail), false); // get column counts response
    addResponseData(respDataOfGettingTVModeDetail, sizeof(respDataOfGettingTVModeDetail), false); // get tv mode response
    addResponseData(respDataOfWriteBFCharset, sizeof(respDataOfWriteBFCharset), false); // update charset with BF
    vcdProfile_t profile;
    profile.video_system = VIDEO_SYSTEM_AUTO; // use auto detect
    memset(&profile, 0, sizeof(vcdProfile_t));
    bool r = rcdeviceOSDInit(&profile);
    EXPECT_EQ(true, r);
}

TEST(RCDeviceTest, TestOSDDeviceSupportWithGetColumnsFailed)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    uint8_t respDataOfGettingColumnsDetail[] = { 0xCC, 0x00, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xFf };
    uint8_t respDataOfWriteDataFailed[] = { 0xCC, 0x01, 0x00, 0x32 };
    uint8_t respDataOfWriteDataSuccess[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responseData, sizeof(responseData), true); // device init response
    addResponseData(respDataOfGettingColumnsDetail, sizeof(respDataOfGettingColumnsDetail), false); // get column counts response
    addResponseData(respDataOfWriteDataSuccess, sizeof(respDataOfWriteDataSuccess), false); // update TV mode with Pal
    addResponseData(respDataOfWriteDataFailed, sizeof(respDataOfWriteDataFailed), false); // update charset with BF
    vcdProfile_t profile;
    memset(&profile, 0, sizeof(vcdProfile_t));
    profile.video_system = VIDEO_SYSTEM_PAL; // use auto detect
    bool r = rcdeviceOSDInit(&profile);
    EXPECT_EQ(false, r);
}

TEST(RCDeviceTest, TestOSDDeviceSupportWithPal)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    uint8_t respDataOfGettingColumnsDetail[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0 };
    uint8_t respDataOfWriteDataSuccess[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responseData, sizeof(responseData), true); // device init response
    addResponseData(respDataOfGettingColumnsDetail, sizeof(respDataOfGettingColumnsDetail), false); // get column counts response
    addResponseData(respDataOfWriteDataSuccess, sizeof(respDataOfWriteDataSuccess), false); // update TV mode with Pal
    addResponseData(respDataOfWriteDataSuccess, sizeof(respDataOfWriteDataSuccess), false); // update charset with BF
    vcdProfile_t profile;
    memset(&profile, 0, sizeof(vcdProfile_t));
    profile.video_system = VIDEO_SYSTEM_PAL; // use auto detect
    bool r = rcdeviceOSDInit(&profile);
    EXPECT_EQ(true, r);
}

TEST(RCDeviceTest, TestOSDDeviceSupportWithWriteTVModeFailed)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    uint8_t respDataOfGettingColumnsDetail[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0 };
    uint8_t respDataOfWriteDataFailed[] = { 0xCC, 0x01, 0x00, 0x32 };
    uint8_t respDataOfWriteDataSuccess[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responseData, sizeof(responseData), true); // device init response
    addResponseData(respDataOfGettingColumnsDetail, sizeof(respDataOfGettingColumnsDetail), false); // get column counts response
    addResponseData(respDataOfWriteDataFailed, sizeof(respDataOfWriteDataFailed), false); // update TV mode with Pal
    addResponseData(respDataOfWriteDataSuccess, sizeof(respDataOfWriteDataSuccess), false); // update charset with BF
    vcdProfile_t profile;
    memset(&profile, 0, sizeof(vcdProfile_t));
    profile.video_system = VIDEO_SYSTEM_PAL; // use auto detect
    bool r = rcdeviceOSDInit(&profile);
    EXPECT_EQ(false, r);
}

TEST(RCDeviceTest, TestOSDDeviceSupportWithWriteCharsetFailed)
{
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x37, 0x00, 0xBD };
    uint8_t respDataOfGettingColumnsDetail[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0 };
    uint8_t respDataOfWriteDataFailed[] = { 0xCC, 0x01, 0x00, 0x32 };
    uint8_t respDataOfWriteDataSuccess[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responseData, sizeof(responseData), true); // device init response
    addResponseData(respDataOfGettingColumnsDetail, sizeof(respDataOfGettingColumnsDetail), false); // get column counts response
    addResponseData(respDataOfWriteDataSuccess, sizeof(respDataOfWriteDataSuccess), false); // update TV mode with Pal
    addResponseData(respDataOfWriteDataFailed, sizeof(respDataOfWriteDataFailed), false); // update charset with BF
    vcdProfile_t profile;
    memset(&profile, 0, sizeof(vcdProfile_t));
    profile.video_system = VIDEO_SYSTEM_PAL; // use auto detect
    bool r = rcdeviceOSDInit(&profile);
    EXPECT_EQ(false, r);
}


TEST(RCDeviceTest, TestDSAGetSettings)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // test getSettings with correct responses
    runcamDeviceSetting_t outSettingList[RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE];
    uint8_t responseDataForGetSettings1[] = { 0xCC, 0x03, 0x18, 0x00, 0x63, 0x68, 0x61, 0x72, 0x73, 0x65, 0x74, 0x00, 0x42, 0x46, 0x00, 0x01, 0x43, 0x4F, 0x4C, 0x55, 0x4D, 0x4E, 0x53, 0x00, 0x33, 0x30, 0x00, 0x22 };
    uint8_t responseDataForGetSettings2[] = { 0xCC, 0x02, 0x23, 0x02, 0x54, 0x56, 0x5F, 0x4D, 0x4F, 0x44, 0x45, 0x00, 0x4E, 0x54, 0x53, 0x43, 0x00, 0x03, 0x53, 0x44, 0x43, 0x41, 0x52, 0x44, 0x5F, 0x43, 0x41, 0x50, 0x41, 0x43, 0x49, 0x54, 0x59, 0x00, 0x30, 0x2F, 0x30, 0x00, 0x9B };
    uint8_t responseDataForGetSettings3[] = { 0xCC, 0x01, 0x33, 0x04, 0x52, 0x45, 0x4D, 0x41, 0x49, 0x4E, 0x49, 0x4E, 0x47, 0x5F, 0x52, 0x45, 0x43, 0x4F, 0x52, 0x44, 0x49, 0x4E, 0x47, 0x5F, 0x54, 0x49, 0x4D, 0x45, 0x00, 0x31, 0x00, 0x05, 0x52, 0x45, 0x53, 0x4F, 0x4C, 0x55, 0x54, 0x49, 0x4F, 0x4E, 0x00, 0x31, 0x30, 0x38, 0x30, 0x70, 0x36, 0x30, 0x66, 0x70, 0x73, 0x00, 0x95 };
    uint8_t responseDataForGetSettings4[] = { 0xCC, 0x00, 0x1F, 0x06, 0x43, 0x41, 0x4D, 0x45, 0x52, 0x41, 0x5F, 0x54, 0x49, 0x4D, 0x45, 0x00, 0x32, 0x30, 0x31, 0x37, 0x31, 0x30, 0x31, 0x30, 0x54, 0x31, 0x37, 0x30, 0x35, 0x31, 0x34, 0x2E, 0x30, 0x00, 0x7C };
    addResponseData(responseDataForGetSettings1, sizeof(responseDataForGetSettings1), true);
    addResponseData(responseDataForGetSettings2, sizeof(responseDataForGetSettings2), false);
    addResponseData(responseDataForGetSettings3, sizeof(responseDataForGetSettings3), false);
    addResponseData(responseDataForGetSettings4, sizeof(responseDataForGetSettings4), false);
    result = runcamDeviceGetSettings(camDevice, 0, (runcamDeviceSetting_t*)outSettingList, RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE);
    uint8_t i = 0;
    printf("get settings:\n");
    while (i < RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE && outSettingList[i].name[0] != 0) {
        printf("%d\t%-20s\t%-20s\n", outSettingList[i].id, outSettingList[i].name, outSettingList[i].value);
        i++;
    }
    EXPECT_EQ(result, true);
    EXPECT_STREQ("charset", outSettingList[0].name);
    EXPECT_STREQ("BF", outSettingList[0].value);
    EXPECT_STREQ("COLUMNS", outSettingList[1].name);
    EXPECT_STREQ("30", outSettingList[1].value);
    EXPECT_STREQ("TV_MODE", outSettingList[2].name);
    EXPECT_STREQ("NTSC", outSettingList[2].value);
    EXPECT_STREQ("SDCARD_CAPACITY", outSettingList[3].name);
    EXPECT_STREQ("0/0", outSettingList[3].value);
    EXPECT_STREQ("REMAINING_RECORDING_1", outSettingList[4].name);
    EXPECT_STREQ("1", outSettingList[4].value);
    EXPECT_STREQ("RESOLUTION", outSettingList[5].name);
    EXPECT_STREQ("1080p60fps", outSettingList[5].value);
    EXPECT_STREQ("CAMERA_TIME", outSettingList[6].name);
    EXPECT_STREQ("20171010T170514.0", outSettingList[6].value);
    clearResponseBuff();

    // test get settings with incorrect response 
    uint8_t responseDataForGetSettings1WithIncorrectCRC[] = { 0xCC, 0x03, 0x18, 0x00, 0x63, 0x68, 0x61, 0x72, 0x73, 0x65, 0x74, 0x00, 0x42, 0x46, 0x00, 0x01, 0x43, 0x4F, 0x4C, 0x55, 0x4D, 0x4E, 0x53, 0x00, 0x33, 0x30, 0x00, 0xA2 };
    addResponseData(responseDataForGetSettings1WithIncorrectCRC, sizeof(responseDataForGetSettings1WithIncorrectCRC), true);
    result = runcamDeviceGetSettings(camDevice, 0, (runcamDeviceSetting_t*)outSettingList, RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test get settings with incomplete response again
    uint8_t responseDataForGetSettings1WithIncorrectLength[] = { 0xCC, 0x03, 0x18, 0x00, 0x63, 0x68, 0x61, 0x72, 0x73, 0x01, 0x43, 0x4F, 0x4C, 0x55, 0x4D, 0x4E, 0x53, 0x00, 0x33, 0x30, 0x00, 0xA2 };
    addResponseData(responseDataForGetSettings1, sizeof(responseDataForGetSettings1), true);
    addResponseData(responseDataForGetSettings1WithIncorrectLength, sizeof(responseDataForGetSettings1WithIncorrectLength), false);
    result = runcamDeviceGetSettings(camDevice, 0, (runcamDeviceSetting_t*)outSettingList, RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test get settings with incorrect length response again
    uint8_t responseDataForGetSettings1WithIncorrectLength2[] = { 0xCC, 0x03, 0x3D, 0x00, 0x63, 0x68, 0x61, 0x72, 0x73, 0x65, 0x74, 0x00, 0x42, 0x46, 0x00, 0x01, 0x43, 0x4F, 0x4C, 0x55, 0x4D, 0x4E, 0x53, 0x00, 0x33, 0x30, 0x00, 0xA2 };
    addResponseData(responseDataForGetSettings1, sizeof(responseDataForGetSettings1), true);
    addResponseData(responseDataForGetSettings1WithIncorrectLength2, sizeof(responseDataForGetSettings1WithIncorrectLength2), false);
    result = runcamDeviceGetSettings(camDevice, 0, (runcamDeviceSetting_t*)outSettingList, RCDEVICE_PROTOCOL_MAX_MENUITEM_PER_PAGE);
    EXPECT_EQ(result, false);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSATextSelectionAccess)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is text selection
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForCharSetDetail1[] = { 0xCC, 0x00, 0x08, 0x09, 0x00, 0x42, 0x46, 0x3B, 0x43, 0x46, 0x00, 0x1C };
    addResponseData(responseDataForCharSetDetail1, sizeof(responseDataForCharSetDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, charsetDetail.value);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_TEXT_SELECTION, charsetDetail.type);
    if (result) {
        printf("setting detail:\n");
        printf("\tcurrentValue = %d\n", charsetDetail.value);
        printf("\tavailable selections = ");
        bool foundCurValue = false;
        for (uint32_t i = 0; i < RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS; i++) {
            if (charsetDetail.textSelections[i].text[0] == 0) {
                break;
            }

            if (i == charsetDetail.value) {
                printf("%s[*], ", charsetDetail.textSelections[i].text);
                foundCurValue = true;
            } else {
                printf("%s, ", charsetDetail.textSelections[i].text);
            }
        }
        printf("\n");
        EXPECT_EQ(foundCurValue, true);
    }
    clearResponseBuff();

    // get setting detail that type is text selection with multiple chunk
    uint8_t responseDataForCharSetDetailChunk1[] = { 0xcc, 0x02, 0x3a, 0x09, 0x02, 0x4f, 0x4e, 0x45, 0x3b, 0x54, 0x57, 0x4f, 0x3b, 0x54, 0x48, 0x52, 0x45, 0x45, 0x3b, 0x46, 0x4f, 0x55, 0x52, 0x3b, 0x46, 0x49, 0x56, 0x45, 0x3b, 0x53, 0x49, 0x58, 0x3b, 0x53, 0x45, 0x56, 0x45, 0x4e, 0x3b, 0x45, 0x49, 0x47, 0x48, 0x54, 0x3b, 0x4e, 0x49, 0x4e, 0x45, 0x3b, 0x54, 0x45, 0x4e, 0x3b, 0x45, 0x6c, 0x65, 0x76, 0x65, 0x6e, 0x3b, 0x8e };
    uint8_t responseDataForCharSetDetailChunk2[] = { 0xcc, 0x01, 0x38, 0x54, 0x77, 0x65, 0x6c, 0x76, 0x65, 0x3b, 0x20, 0x74, 0x68, 0x69, 0x72, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x66, 0x6f, 0x75, 0x72, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x66, 0x69, 0x66, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x73, 0x69, 0x78, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x73, 0x65, 0x76, 0x65, 0x6e, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x96 };
    uint8_t responseDataForCharSetDetailChunk3[] = { 0xcc, 0x00, 0x34, 0x45, 0x69, 0x67, 0x68, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x4e, 0x69, 0x6e, 0x65, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x74, 0x77, 0x65, 0x6e, 0x74, 0x79, 0x3b, 0x20, 0x74, 0x77, 0x65, 0x6e, 0x74, 0x79, 0x20, 0x6f, 0x6e, 0x65, 0x3b, 0x20, 0x74, 0x77, 0x65, 0x6e, 0x74, 0x79, 0x20, 0x74, 0x77, 0x6f, 0x3b, 0x00, 0x31 };
    addResponseData(responseDataForCharSetDetailChunk1, sizeof(responseDataForCharSetDetailChunk1), true);
    addResponseData(responseDataForCharSetDetailChunk2, sizeof(responseDataForCharSetDetailChunk2), false);
    addResponseData(responseDataForCharSetDetailChunk3, sizeof(responseDataForCharSetDetailChunk3), false);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(2, charsetDetail.value);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_TEXT_SELECTION, charsetDetail.type);
    if (result) {
        printf("setting detail:\n");
        printf("\tcurrentValue = %d\n", charsetDetail.value);
        printf("\tavailable selections = ");
        bool foundCurValue = false;
        for (uint32_t i = 0; i < RCDEVICE_PROTOCOL_MAX_TEXT_SELECTIONS; i++) {
            if (charsetDetail.textSelections[i].text[0] == 0) {
                break;
            }

            if (i == charsetDetail.value) {
                printf("%s[*], ", charsetDetail.textSelections[i].text);
                foundCurValue = true;
            } else {
                printf("%s, ", charsetDetail.textSelections[i].text);
            }
        }
        printf("\n");
        EXPECT_EQ(foundCurValue, true);
    }
    clearResponseBuff();

    // test get setting detail that type is textselection, but the response is incorrect
    uint8_t incorrectResponseDataForCharSetDetail1[] = { 0xCC, 0x00, 0x08, 0x09, 0x00, 0x42, 0x46, 0x3B, 0x43, 0x46, 0x00, 0x1A };
    addResponseData(incorrectResponseDataForCharSetDetail1, sizeof(incorrectResponseDataForCharSetDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test get setting detail that type is textselection, but the response length is incorrect
    uint8_t incorrectResponseDataForCharSetDetail2[] = { 0xCC, 0x00, 0x08, 0x09, 0x00, 0x42 };
    addResponseData(incorrectResponseDataForCharSetDetail2, sizeof(incorrectResponseDataForCharSetDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test get setting detail that type is textselection multiple chunk, but the response length is incorrect
    uint8_t incorrectResponseDataForCharSetDetailChunk2[] = { 0xcc, 0x01, 0x38, 0x54, 0x77, 0x65, 0x6c, 0x76, 0x65, 0x3b, 0x20, 0x74, 0x68, 0x69, 0x72, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x66, 0x6f, 0x75, 0x72, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x66, 0x69, 0x66, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x73, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x20, 0x73, 0x65, 0x76, 0x65, 0x6e, 0x74, 0x65, 0x65, 0x6e, 0x3b, 0x96 };
    addResponseData(responseDataForCharSetDetailChunk1, sizeof(responseDataForCharSetDetailChunk1), true);
    addResponseData(incorrectResponseDataForCharSetDetailChunk2, sizeof(incorrectResponseDataForCharSetDetailChunk2), false);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // test update setting
    runcamDeviceWriteSettingResponse_t updateSettingResponse;
    uint8_t responweDataForUpdateCharSet1[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responweDataForUpdateCharSet1, sizeof(responweDataForUpdateCharSet1), true);
    uint8_t newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that needUpdatemenuItems's value is 1
    uint8_t responweDataForUpdateCharSet2[] = { 0xCC, 0x00, 0x01, 0xec };
    addResponseData(responweDataForUpdateCharSet2, sizeof(responweDataForUpdateCharSet2), true);
    newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(1, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that resultCode's value is 1
    uint8_t responweDataForUpdateCharSet3[] = { 0xCC, 0x01, 0x00, 0x32 };
    addResponseData(responweDataForUpdateCharSet3, sizeof(responweDataForUpdateCharSet3), true);
    newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_CHARSET, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(1, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSAUint8AccessProtocol)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is uint8
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0 };
    addResponseData(responseDataForColumnsDetail1, sizeof(responseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_UINT8, charsetDetail.type);
    EXPECT_EQ(30, charsetDetail.value);
    EXPECT_EQ(30, charsetDetail.minValue);
    EXPECT_EQ(30, charsetDetail.maxValue);
    EXPECT_EQ(1, charsetDetail.stepSize);
    clearResponseBuff();

    // get setting detail that type is uint8, but the response length is incorrect
    uint8_t incorrectResponseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E };
    addResponseData(incorrectResponseDataForColumnsDetail1, sizeof(incorrectResponseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is uint8, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail2[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xC2 };
    addResponseData(incorrectResponseDataForColumnsDetail2, sizeof(incorrectResponseDataForColumnsDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is uint8, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail3[] = { 0xCC, 0x00, 0x05, 0x00, 0x1E, 0x1E, 0x1E, 0x01, 0xB0, 0xAA, 0xBB, 0xCC };
    addResponseData(incorrectResponseDataForColumnsDetail3, sizeof(incorrectResponseDataForColumnsDetail3), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // test update setting
    runcamDeviceWriteSettingResponse_t updateSettingResponse;
    uint8_t responweDataForUpdateCharSet1[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responweDataForUpdateCharSet1, sizeof(responweDataForUpdateCharSet1), true);
    uint8_t newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that needUpdatemenuItems's value is 1
    uint8_t responweDataForUpdateCharSet2[] = { 0xCC, 0x00, 0x01, 0xec };
    addResponseData(responweDataForUpdateCharSet2, sizeof(responweDataForUpdateCharSet2), true);
    newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(1, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that resultCode's value is 1
    uint8_t responweDataForUpdateCharSet3[] = { 0xCC, 0x01, 0x00, 0x32 };
    addResponseData(responweDataForUpdateCharSet3, sizeof(responweDataForUpdateCharSet3), true);
    newValue = 1;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &newValue, sizeof(uint8_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(1, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSAUint16AccessProtocol)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is uint16
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForColumnsDetail1[] = { 0xcc, 0x00, 0x09, 0x02, 0x1e, 0x3a, 0x1e, 0x00, 0xff, 0xff, 0x64, 0x00, 0x6c };
    addResponseData(responseDataForColumnsDetail1, sizeof(responseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_UINT16, charsetDetail.type);
    EXPECT_EQ(14878, charsetDetail.value);
    EXPECT_EQ(30, charsetDetail.minValue);
    EXPECT_EQ(65535, charsetDetail.maxValue);
    EXPECT_EQ(100, charsetDetail.stepSize);
    clearResponseBuff();

    // get setting detail that type is uint16, but the response length is incorrect
    uint8_t incorrectResponseDataForColumnsDetail1[] = { 0xcc, 0x00, 0x09, 0x02, 0x1e, 0x3a, 0x1e, 0x00, 0xff, 0xff };
    addResponseData(incorrectResponseDataForColumnsDetail1, sizeof(incorrectResponseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is uint16, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail2[] = { 0xcc, 0x00, 0x09, 0x02, 0x1e, 0x3a, 0x1e, 0x00, 0xff, 0xff, 0x64, 0x00, 0x7c };
    addResponseData(incorrectResponseDataForColumnsDetail2, sizeof(incorrectResponseDataForColumnsDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is uint16, but the response has some randome data at trailing
    uint8_t incorrectResponseDataForColumnsDetail3[] = { 0xcc, 0x00, 0x09, 0x02, 0x1e, 0x3a, 0x1e, 0x00, 0xff, 0xff, 0x64, 0x00, 0x6c, 0xAA, 0xBB, 0xCC };
    addResponseData(incorrectResponseDataForColumnsDetail3, sizeof(incorrectResponseDataForColumnsDetail3), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // test update setting
    runcamDeviceWriteSettingResponse_t updateSettingResponse;
    uint8_t responweDataForUpdateCharSet1[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responweDataForUpdateCharSet1, sizeof(responweDataForUpdateCharSet1), true);
    uint16_t newValue = 55;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)&newValue, sizeof(uint16_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that needUpdatemenuItems's value is 1
    uint8_t responweDataForUpdateCharSet2[] = { 0xCC, 0x00, 0x01, 0xec };
    addResponseData(responweDataForUpdateCharSet2, sizeof(responweDataForUpdateCharSet2), true);
    newValue = 190;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)&newValue, sizeof(uint16_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(1, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that resultCode's value is 1
    uint8_t responweDataForUpdateCharSet3[] = { 0xCC, 0x01, 0x00, 0x32 };
    addResponseData(responweDataForUpdateCharSet3, sizeof(responweDataForUpdateCharSet3), true);
    newValue = 10241;
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)&newValue, sizeof(uint16_t), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(1, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSAFloatAccessProtocol)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is float
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForColumnsDetail1[] = { 0xcc, 0x00, 0x12, 0x08, 0x1e, 0x33, 0x00, 0x00, 0x3a, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x04, 0xf4, 0x01, 0x00, 0x00, 0xc9 };
    addResponseData(responseDataForColumnsDetail1, sizeof(responseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_FLOAT, charsetDetail.type);
    EXPECT_EQ(13086, charsetDetail.value);
    EXPECT_EQ(58, charsetDetail.minValue);
    EXPECT_EQ(4294967295, charsetDetail.maxValue);
    EXPECT_EQ(4, charsetDetail.decimalPoint);
    EXPECT_EQ(500, charsetDetail.stepSize);
    clearResponseBuff();

    // get setting detail that type is float, but the response length is incorrect
    uint8_t incorrectResponseDataForColumnsDetail1[] = { 0xcc, 0x00, 0x07, 0x08, 0x1e, 0x3a, 0xff, 0x0a };
    addResponseData(incorrectResponseDataForColumnsDetail1, sizeof(incorrectResponseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is float, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail2[] = { 0xcc, 0x00, 0x07, 0x08, 0x1e, 0x3a, 0xff, 0x0a, 0x00, 0x01, 0xf2 };
    addResponseData(incorrectResponseDataForColumnsDetail2, sizeof(incorrectResponseDataForColumnsDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is float, but the response has some randome data at trailing
    uint8_t incorrectResponseDataForColumnsDetail3[] = { 0xcc, 0x00, 0x12, 0x08, 0x1e, 0x33, 0x00, 0x00, 0x3a, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x04, 0xf4, 0x01, 0x00, 0x00, 0xc9, 0xAA, 0xBB, 0xCC };
    addResponseData(incorrectResponseDataForColumnsDetail3, sizeof(incorrectResponseDataForColumnsDetail3), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // test update setting
    runcamDeviceWriteSettingResponse_t updateSettingResponse;
    uint8_t responweDataForUpdateCharSet1[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responweDataForUpdateCharSet1, sizeof(responweDataForUpdateCharSet1), true);
    uint16_t newValue[5] = { 0x1E, 0x1E, 0x00, 0x00, 0x2 }; // the data struct is UINT32_T + UINT8(Decimal Point)
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that needUpdatemenuItems's value is 1
    uint8_t responweDataForUpdateCharSet2[] = { 0xCC, 0x00, 0x01, 0xec };
    addResponseData(responweDataForUpdateCharSet2, sizeof(responweDataForUpdateCharSet2), true);
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(1, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that resultCode's value is 1
    uint8_t responweDataForUpdateCharSet3[] = { 0xCC, 0x01, 0x00, 0x32 };
    addResponseData(responweDataForUpdateCharSet3, sizeof(responweDataForUpdateCharSet3), true);
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(1, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSAStringAccessProtocol)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is string
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x14, 0x0A, 0x31, 0x39, 0x37, 0x32, 0x30, 0x32, 0x31, 0x36, 0x54, 0x31, 0x39, 0x31, 0x35, 0x33, 0x32, 0x2E, 0x30, 0x00, 0x14, 0x20 };
    addResponseData(responseDataForColumnsDetail1, sizeof(responseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_STRING, charsetDetail.type);
    EXPECT_STREQ("19720216T191532.0", charsetDetail.stringValue);
    EXPECT_EQ(20, charsetDetail.maxStringSize);
    clearResponseBuff();

    // get setting detail that type is string, but the response length is incorrect
    uint8_t incorrectResponseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x14, 0x0A, 0x31, 0x39, 0x37, 0x32, 0x30, 0x32, 0x31, 0x36, 0x54, 0x31, 0x39, 0x31, 0x35, 0x33, 0x32, 0x2E, 0x30 };
    addResponseData(incorrectResponseDataForColumnsDetail1, sizeof(incorrectResponseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is string, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail2[] = { 0xCC, 0x00, 0x14, 0x0A, 0x31, 0x39, 0x37, 0x32, 0x30, 0x32, 0x31, 0x36, 0x54, 0x31, 0x39, 0x31, 0x35, 0x33, 0x32, 0x2E, 0x30, 0x00, 0x14, 0x21 };
    addResponseData(incorrectResponseDataForColumnsDetail2, sizeof(incorrectResponseDataForColumnsDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is string, but the response has some randome data at trailing
    uint8_t incorrectResponseDataForColumnsDetail3[] = { 0xCC, 0x00, 0x14, 0x0A, 0x31, 0x39, 0x37, 0x32, 0x30, 0x32, 0x31, 0x36, 0x54, 0x31, 0x39, 0x31, 0x35, 0x33, 0x32, 0x2E, 0x30, 0x00, 0x14, 0x20, 0xAA, 0xBB, 0xCC };
    addResponseData(incorrectResponseDataForColumnsDetail3, sizeof(incorrectResponseDataForColumnsDetail3), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // test update setting
    runcamDeviceWriteSettingResponse_t updateSettingResponse;
    uint8_t responweDataForUpdateCharSet1[] = { 0xCC, 0x00, 0x00, 0x39 };
    addResponseData(responweDataForUpdateCharSet1, sizeof(responweDataForUpdateCharSet1), true);
    uint16_t newValue[] = { 0x31, 0x39, 0x37, 0x32, 0x30, 0x32, 0x31, 0x36, 0x54, 0x31, 0x39, 0x31, 0x35, 0x33, 0x32, 0x2E, 0x30, 0x00 }; // the data struct is UINT32_T + UINT8(Decimal Point)
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that needUpdatemenuItems's value is 1
    uint8_t responweDataForUpdateCharSet2[] = { 0xCC, 0x00, 0x01, 0xec };
    addResponseData(responweDataForUpdateCharSet2, sizeof(responweDataForUpdateCharSet2), true);
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(0, updateSettingResponse.resultCode);
    EXPECT_EQ(1, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();

    // test update setting with response that resultCode's value is 1
    uint8_t responweDataForUpdateCharSet3[] = { 0xCC, 0x01, 0x00, 0x32 };
    addResponseData(responweDataForUpdateCharSet3, sizeof(responweDataForUpdateCharSet3), true);
    result = runcamDeviceWriteSetting(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, (uint8_t*)newValue, sizeof(newValue), &updateSettingResponse);
    EXPECT_EQ(result, true);
    EXPECT_EQ(1, updateSettingResponse.resultCode);
    EXPECT_EQ(0, updateSettingResponse.needUpdateMenuItems);
    clearResponseBuff();
}

TEST(RCDeviceTest, TestDSAInfoAccessProtocol)
{
    // init device that have not 5 key OSD cable simulation feature
    memset(&testData, 0, sizeof(testData));
    testData.isRunCamSplitOpenPortSupported = true;
    testData.isRunCamSplitPortConfigurated = true;
    testData.isAllowBufferReadWrite = true;
    testData.maxTimesOfRespDataAvailable = 0;
    uint8_t responseData[] = { 0xCC, 0x01, 0x3F, 0x00, 0xE5 };
    addResponseData(responseData, sizeof(responseData), true);
    bool result = rcdeviceInit();
    EXPECT_EQ(result, true);
    clearResponseBuff();

    // get setting detail that type is string
    runcamDeviceSettingDetail_t charsetDetail;
    uint8_t responseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x0B, 0x0C, 0x31, 0x31, 0x35, 0x2F, 0x36, 0x30, 0x38, 0x39, 0x30, 0x00, 0xFE };
    addResponseData(responseDataForColumnsDetail1, sizeof(responseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    EXPECT_EQ(RCDEVICE_PROTOCOL_SETTINGTYPE_INFO, charsetDetail.type);
    EXPECT_STREQ("115/60890", charsetDetail.stringValue);
    clearResponseBuff();

    // get setting detail that type is string, but the response length is incorrect
    uint8_t incorrectResponseDataForColumnsDetail1[] = { 0xCC, 0x00, 0x0B, 0x0C, 0x31, 0x31, 0x35, 0x2F, 0x36, 0x30, 0x38 };
    addResponseData(incorrectResponseDataForColumnsDetail1, sizeof(incorrectResponseDataForColumnsDetail1), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is string, but the response crc is incorrect
    uint8_t incorrectResponseDataForColumnsDetail2[] = { 0xCC, 0x00, 0x0B, 0x0C, 0x31, 0x31, 0x35, 0x2F, 0x36, 0x30, 0x38, 0x39, 0x30, 0x00, 0xFF };
    addResponseData(incorrectResponseDataForColumnsDetail2, sizeof(incorrectResponseDataForColumnsDetail2), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, false);
    clearResponseBuff();

    // get setting detail that type is string, but the response has some randome data at trailing
    uint8_t incorrectResponseDataForColumnsDetail3[] = { 0xCC, 0x00, 0x0B, 0x0C, 0x31, 0x31, 0x35, 0x2F, 0x36, 0x30, 0x38, 0x39, 0x30, 0x00, 0xFE, 0xAA, 0xBB, 0xCC };
    addResponseData(incorrectResponseDataForColumnsDetail3, sizeof(incorrectResponseDataForColumnsDetail3), true);
    result = runcamDeviceGetSettingDetail(camDevice, RCDEVICE_PROTOCOL_SETTINGID_DISP_COLUMNS, &charsetDetail);
    EXPECT_EQ(result, true);
    clearResponseBuff();
}

extern "C" {
    serialPort_t *openSerialPort(serialPortIdentifier_e identifier, serialPortFunction_e functionMask, serialReceiveCallbackPtr callback, void *callbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
    {
        UNUSED(identifier);
        UNUSED(functionMask);
        UNUSED(baudRate);
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
            s.rxCallback = callback;
            s.rxCallbackData = callbackData;
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
            portConfig.blackbox_baudrateIndex = BAUD_115200;
            portConfig.functionMask = FUNCTION_MSP;

            return &portConfig;
        }

        return NULL;
    }

    uint32_t serialRxBytesWaiting(const serialPort_t *instance) 
    { 
        UNUSED(instance);

        uint8_t bufIndex = testData.indexOfCurrentRespBuf;
        uint8_t leftDataLen = 0;
        if (testData.responseDataReadPos >= testData.responseBufsLen[bufIndex]) {
            return 0;
        } else {
            leftDataLen = testData.responseBufsLen[bufIndex] - testData.responseDataReadPos;
        }

        if (leftDataLen) {
            return leftDataLen;
        }

        return 0;
    }

    uint8_t serialRead(serialPort_t *instance) 
    { 
        UNUSED(instance);

        uint8_t bufIndex = testData.indexOfCurrentRespBuf;
        uint8_t *buffer = NULL;
        uint8_t leftDataLen = 0;
        if (testData.responseDataReadPos >= testData.responseBufsLen[bufIndex]) {
            leftDataLen = 0;
        } else {
            buffer = testData.responesBufs[bufIndex];
            leftDataLen = testData.responseBufsLen[bufIndex] - testData.responseDataReadPos;
        }

        if (leftDataLen) {
            return buffer[testData.responseDataReadPos++];
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

    uint8_t sbufReadU8(sbuf_t *src)
    {
        if (testData.isAllowBufferReadWrite) {
            return *src->ptr++;
        }

        return 0;
    }

    void sbufAdvance(sbuf_t *buf, int size)
    {
        if (testData.isAllowBufferReadWrite) {
            buf->ptr += size;
        }
    }

    int sbufBytesRemaining(sbuf_t *buf)
    {
        if (testData.isAllowBufferReadWrite) {
            return buf->end - buf->ptr;
        }
        return 0;
    }

    const uint8_t* sbufConstPtr(const sbuf_t *buf)
    {
        return buf->ptr;
    }

    void sbufReadData(sbuf_t *src, void *data, int len)
    {
        if (testData.isAllowBufferReadWrite) {
            memcpy(data, src->ptr, len);
        }
    }

    uint16_t sbufReadU16(sbuf_t *src)
    {
        uint16_t ret;
        ret = sbufReadU8(src);
        ret |= sbufReadU8(src) << 8;
        return ret;
    }

    void sbufWriteU16(sbuf_t *dst, uint16_t val)
    {
        sbufWriteU8(dst, val >> 0);
        sbufWriteU8(dst, val >> 8);
    }

    void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val)
    {
        sbufWriteU8(dst, val >> 8);
        sbufWriteU8(dst, (uint8_t)val);
    }

    bool feature(uint32_t) { return false; }

    void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count) 
    { 
        UNUSED(instance); UNUSED(data); UNUSED(count); 

        // // reset the input buffer
        testData.responseDataReadPos = 0;
        testData.indexOfCurrentRespBuf++;
        if (testData.indexOfCurrentRespBuf >= testData.responseBufCount) {
            testData.indexOfCurrentRespBuf = 0;
        }
        // testData.maxTimesOfRespDataAvailable = testData.responseDataLen + 1;
    }

    serialPortConfig_t *findNextSerialPortConfig(serialPortFunction_e function)
    {
        UNUSED(function);

        return NULL;
    }

    void closeSerialPort(serialPort_t *serialPort)
    {
        UNUSED(serialPort);
    }

    uint8_t* sbufPtr(sbuf_t *buf)
    {
        return buf->ptr;
    }

    uint32_t sbufReadU32(sbuf_t *src)
    {
        uint32_t ret;
        ret = sbufReadU8(src);
        ret |= sbufReadU8(src) <<  8;
        ret |= sbufReadU8(src) << 16;
        ret |= sbufReadU8(src) << 24;
        return ret;
    }
    

    uint32_t millis(void) { return testData.millis++; }

    void beeper(beeperMode_e mode) { UNUSED(mode); }
    uint8_t armingFlags = 0;
    bool cmsInMenu;
    uint32_t resumeRefreshAt = 0;
    int getArmingDisableFlags(void) {return 0;}
}
