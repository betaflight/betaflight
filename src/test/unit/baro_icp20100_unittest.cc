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

#include <stdint.h>
#include <string.h>

extern "C" {

#include "platform.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_icp20100.h"
#include "drivers/bus.h"

bool icp20100VersionSupported(uint8_t version);
bool icp20100VersionRequiresOtp(uint8_t version, uint8_t bootStatus);
int32_t icp20100SignExtend20(uint32_t value);
void icp20100DecodeFrame(const uint8_t data[6], int32_t *rawPressure, int32_t *rawTemperature);
void icp20100ConvertRaw(int32_t rawPressure, int32_t rawTemperature, int32_t *pressure, int32_t *temperature);
bool icp20100AcceptSample(uint8_t *samplesToDiscard);
bool icp20100LoadOtpTrim(const extDevice_t *dev, uint8_t version);
bool icp20100ReadUP(baroDev_t *baro);
bool icp20100GetUP(baroDev_t *baro);

extern int32_t icp20100RawPressure;
extern int32_t icp20100RawTemperature;
extern bool icp20100SampleValid;
extern uint8_t icp20100SamplesToDiscard;

}

#include "gtest/gtest.h"

typedef struct {
    uint8_t reg;
    uint8_t value;
} registerWrite_t;

static uint8_t simulatedRegisters[256];
static unsigned registerReadCount[256];
static registerWrite_t registerWrites[64];
static unsigned registerWriteCount;
static bool simulatedBusBusy;
static bool simulatedBusError;
static bool failAsyncReadStart;
static bool failDummyRead;
static int failReadRegister;
static int failWriteRegister;
static uint8_t asyncReadRegister;
static uint8_t asyncReadLength;
static uint8_t fifoData[16 * 6];
static unsigned dummyReadCount;
static unsigned registeredDeviceCount;
static busDevice_t simulatedBus;
static baroDev_t simulatedBaro;

static void resetSimulatedDevice(void)
{
    memset(simulatedRegisters, 0, sizeof(simulatedRegisters));
    memset(registerReadCount, 0, sizeof(registerReadCount));
    memset(registerWrites, 0, sizeof(registerWrites));
    registerWriteCount = 0;
    simulatedBusBusy = false;
    simulatedBusError = false;
    failAsyncReadStart = false;
    failDummyRead = false;
    failReadRegister = -1;
    failWriteRegister = -1;
    asyncReadRegister = 0;
    asyncReadLength = 0;
    memset(fifoData, 0, sizeof(fifoData));
    dummyReadCount = 0;
    registeredDeviceCount = 0;
    memset(&simulatedBus, 0, sizeof(simulatedBus));
    memset(&simulatedBaro, 0, sizeof(simulatedBaro));
    simulatedBus.busType = BUS_TYPE_I2C;
    simulatedBaro.dev.bus = &simulatedBus;

    simulatedRegisters[0x05] = 0xC0;
    simulatedRegisters[0x06] = 0x80;
    simulatedRegisters[0x07] = 0x8A;
    simulatedRegisters[0xBF] = 0xF0;
    simulatedRegisters[0xCD] = 0x01;

    icp20100RawPressure = 0;
    icp20100RawTemperature = 0;
    icp20100SampleValid = false;
    icp20100SamplesToDiscard = 0;
}

static void encodeFrame(uint8_t *frame, int32_t pressure, int32_t temperature)
{
    const uint32_t encodedPressure = static_cast<uint32_t>(pressure) & 0xFFFFF;
    const uint32_t encodedTemperature = static_cast<uint32_t>(temperature) & 0xFFFFF;
    frame[0] = encodedPressure;
    frame[1] = encodedPressure >> 8;
    frame[2] = encodedPressure >> 16;
    frame[3] = encodedTemperature;
    frame[4] = encodedTemperature >> 8;
    frame[5] = encodedTemperature >> 16;
}

static int findRegisterWrite(uint8_t reg, uint8_t value)
{
    for (unsigned i = 0; i < registerWriteCount; i++) {
        if (registerWrites[i].reg == reg && registerWrites[i].value == value) {
            return i;
        }
    }
    return -1;
}

TEST(baroIcp20100Test, RecognizesDocumentedAsicVersions)
{
    EXPECT_TRUE(icp20100VersionSupported(0x00));
    EXPECT_TRUE(icp20100VersionSupported(0xB2));
    EXPECT_FALSE(icp20100VersionSupported(0x01));
    EXPECT_FALSE(icp20100VersionSupported(0xB1));

    EXPECT_TRUE(icp20100VersionRequiresOtp(0x00, 0xF0));
    EXPECT_FALSE(icp20100VersionRequiresOtp(0x00, 0xF1));
    EXPECT_FALSE(icp20100VersionRequiresOtp(0xB2, 0x00));
}

TEST(baroIcp20100Test, SignExtendsTwentyBitSamples)
{
    EXPECT_EQ(0, icp20100SignExtend20(0x00000));
    EXPECT_EQ(524287, icp20100SignExtend20(0x7FFFF));
    EXPECT_EQ(-524288, icp20100SignExtend20(0x80000));
    EXPECT_EQ(-1, icp20100SignExtend20(0xFFFFF));
}

TEST(baroIcp20100Test, DecodesPressureFirstFifoFrame)
{
    // Pressure -131072 (0xE0000), temperature +262144 (0x40000).
    const uint8_t frame[6] = { 0x00, 0x00, 0xFE, 0x00, 0x00, 0xA4 };
    int32_t rawPressure;
    int32_t rawTemperature;

    icp20100DecodeFrame(frame, &rawPressure, &rawTemperature);

    EXPECT_EQ(-131072, rawPressure);
    EXPECT_EQ(262144, rawTemperature);
}

TEST(baroIcp20100Test, ConvertsOfficialFormulaAnchorPoints)
{
    int32_t pressure;
    int32_t temperature;

    icp20100ConvertRaw(0, 0, &pressure, &temperature);
    EXPECT_EQ(70000, pressure);
    EXPECT_EQ(2500, temperature);

    icp20100ConvertRaw(-131072, -262144, &pressure, &temperature);
    EXPECT_EQ(30000, pressure);
    EXPECT_EQ(-4000, temperature);

    icp20100ConvertRaw(131072, 262144, &pressure, &temperature);
    EXPECT_EQ(110000, pressure);
    EXPECT_EQ(9000, temperature);
}

TEST(baroIcp20100Test, DiscardsFirSettlingSamplesThenAcceptsData)
{
    uint8_t samplesToDiscard = 2;

    EXPECT_FALSE(icp20100AcceptSample(&samplesToDiscard));
    EXPECT_EQ(1, samplesToDiscard);
    EXPECT_FALSE(icp20100AcceptSample(&samplesToDiscard));
    EXPECT_EQ(0, samplesToDiscard);
    EXPECT_TRUE(icp20100AcceptSample(&samplesToDiscard));
}

TEST(baroIcp20100Test, LoadsVersionATrimFromOtpInDocumentedOrder)
{
    resetSimulatedDevice();
    const extDevice_t dev = {};

    ASSERT_TRUE(icp20100LoadOtpTrim(&dev, 0x00));

    // F8 offset[5:0], F9 gain[2:0], and FA HFOSC[6:0].
    EXPECT_EQ(0xEA, simulatedRegisters[0x05]);
    EXPECT_EQ(0xD5, simulatedRegisters[0x06]);
    EXPECT_EQ(0xDA, simulatedRegisters[0x07]);
    EXPECT_EQ(0xF1, simulatedRegisters[0xBF]);
    EXPECT_EQ(0x00, simulatedRegisters[0xBE]);
    EXPECT_EQ(0x00, simulatedRegisters[0xC0]);

    const int otpEnabled = findRegisterWrite(0xAC, 0x03);
    const int otpDisabled = findRegisterWrite(0xAC, 0x00);
    const int trimWritten = findRegisterWrite(0x05, 0xEA);
    const int registersLocked = findRegisterWrite(0xBE, 0x00);
    const int standbySelected = findRegisterWrite(0xC0, 0x00);

    ASSERT_GE(otpEnabled, 0);
    ASSERT_GE(otpDisabled, 0);
    ASSERT_GE(trimWritten, 0);
    ASSERT_GE(registersLocked, 0);
    ASSERT_GE(standbySelected, 0);
    EXPECT_LT(otpEnabled, otpDisabled);
    EXPECT_LT(otpDisabled, trimWritten);
    EXPECT_LT(trimWritten, registersLocked);
    EXPECT_LT(registersLocked, standbySelected);
}

TEST(baroIcp20100Test, SkipsOtpForVersionBAndAlreadyBootedVersionA)
{
    resetSimulatedDevice();
    const extDevice_t dev = {};

    EXPECT_TRUE(icp20100LoadOtpTrim(&dev, 0xB2));
    EXPECT_EQ(0U, registerWriteCount);

    resetSimulatedDevice();
    simulatedRegisters[0xBF] = 0xF1;
    EXPECT_TRUE(icp20100LoadOtpTrim(&dev, 0x00));
    EXPECT_EQ(0U, registerWriteCount);

    EXPECT_FALSE(icp20100LoadOtpTrim(&dev, 0xB1));
}

TEST(baroIcp20100Test, DetectConfiguresMode2AtDrainSafeCadence)
{
    resetSimulatedDevice();
    simulatedRegisters[0x0C] = 0x63;
    simulatedRegisters[0xD3] = 0xB2;

    ASSERT_TRUE(icp20100Detect(&simulatedBaro));
    EXPECT_EQ(0x63, simulatedBaro.dev.busType_u.i2c.address);
    EXPECT_EQ(22000, simulatedBaro.up_delay);
    EXPECT_TRUE(simulatedBaro.combined_read);
    EXPECT_EQ(1U, registeredDeviceCount);
    EXPECT_GE(findRegisterWrite(0xC4, 0x80), 0);
    EXPECT_GE(findRegisterWrite(0xC0, 0x48), 0);
}

TEST(baroIcp20100Test, DetectionFailureRestoresDefaultAddressAndDoesNotRegister)
{
    resetSimulatedDevice();
    simulatedRegisters[0x0C] = 0x00;

    EXPECT_FALSE(icp20100Detect(&simulatedBaro));
    EXPECT_EQ(0, simulatedBaro.dev.busType_u.i2c.address);
    EXPECT_EQ(0U, registeredDeviceCount);
}

TEST(baroIcp20100Test, DrainsFifoBurstAndKeepsNewestFrame)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 3;
    encodeFrame(&fifoData[0], 100, 200);
    encodeFrame(&fifoData[6], 300, 400);
    encodeFrame(&fifoData[12], -500, -600);

    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    EXPECT_EQ(0xFA, asyncReadRegister);
    EXPECT_EQ(18, asyncReadLength);
    EXPECT_FALSE(icp20100SampleValid);

    ASSERT_TRUE(icp20100GetUP(&simulatedBaro));
    EXPECT_EQ(-500, icp20100RawPressure);
    EXPECT_EQ(-600, icp20100RawTemperature);
    EXPECT_TRUE(icp20100SampleValid);
    EXPECT_EQ(1U, dummyReadCount);
}

TEST(baroIcp20100Test, BurstAdvancesSettlingByEveryDrainedFrame)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 3;
    encodeFrame(&fifoData[12], 1234, 5678);
    icp20100SamplesToDiscard = 2;

    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    ASSERT_TRUE(icp20100GetUP(&simulatedBaro));

    EXPECT_EQ(0, icp20100SamplesToDiscard);
    EXPECT_TRUE(icp20100SampleValid);
    EXPECT_EQ(1234, icp20100RawPressure);
}

TEST(baroIcp20100Test, DrainsDocumentedMaximumFifoLevelInOneBurst)
{
    resetSimulatedDevice();
    // FIFO_FULL is asserted together with the documented maximum level.
    simulatedRegisters[0xC4] = 0x30;
    encodeFrame(&fifoData[15 * 6], -12345, 23456);

    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    EXPECT_EQ(sizeof(fifoData), asyncReadLength);
    ASSERT_TRUE(icp20100GetUP(&simulatedBaro));
    EXPECT_EQ(-12345, icp20100RawPressure);
    EXPECT_EQ(23456, icp20100RawTemperature);
}

TEST(baroIcp20100Test, EmptyFifoRetriesWithoutStartingAsyncRead)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 0x40;

    EXPECT_FALSE(icp20100ReadUP(&simulatedBaro));
    EXPECT_EQ(0, asyncReadLength);
    EXPECT_EQ(1U, dummyReadCount);
}

TEST(baroIcp20100Test, OverflowIsClearedAndAvailableFramesAreDrained)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 2;
    simulatedRegisters[0xC1] = 0x01;
    encodeFrame(&fifoData[6], 123, 456);

    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    EXPECT_EQ(12, asyncReadLength);
    EXPECT_GE(findRegisterWrite(0xC1, 0x01), 0);
    ASSERT_TRUE(icp20100GetUP(&simulatedBaro));
    EXPECT_EQ(123, icp20100RawPressure);
    EXPECT_EQ(456, icp20100RawTemperature);
    EXPECT_TRUE(icp20100SampleValid);
}

TEST(baroIcp20100Test, InvalidFifoStateIsRejectedWithoutBlockingReconfiguration)
{
    const uint8_t invalidFifoFill[] = {
        0x11, // Level above the 16-frame FIFO capacity.
        0x41, // EMPTY contradicts a nonzero level.
        0x21, // FULL contradicts a level below 16.
        0x10, // Level 16 without FULL is also contradictory.
    };

    for (const uint8_t fifoFill : invalidFifoFill) {
        resetSimulatedDevice();
        simulatedRegisters[0xC4] = fifoFill;
        // If runtime recovery accidentally polls MODE_SYNC, it would time out.
        simulatedRegisters[0xCD] = 0x00;
        icp20100SampleValid = true;

        EXPECT_FALSE(icp20100ReadUP(&simulatedBaro));
        EXPECT_FALSE(icp20100SampleValid);
        EXPECT_EQ(14, icp20100SamplesToDiscard);
        EXPECT_EQ(0, asyncReadLength);
        EXPECT_EQ(0U, registerReadCount[0xCD]);
        EXPECT_LT(findRegisterWrite(0xC4, 0x80), 0);
        EXPECT_LT(findRegisterWrite(0xC0, 0x48), 0);
        EXPECT_EQ(1U, dummyReadCount);
    }
}

TEST(baroIcp20100Test, BusyReadAndGetRetryWithoutPublishing)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 1;
    simulatedBusBusy = true;

    EXPECT_FALSE(icp20100ReadUP(&simulatedBaro));
    EXPECT_FALSE(icp20100GetUP(&simulatedBaro));
    EXPECT_EQ(0, asyncReadLength);
    EXPECT_EQ(0U, dummyReadCount);
}

TEST(baroIcp20100Test, AsyncStartFailurePerformsRequiredDummyRead)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 1;
    failAsyncReadStart = true;

    EXPECT_FALSE(icp20100ReadUP(&simulatedBaro));
    EXPECT_EQ(1U, dummyReadCount);
    EXPECT_FALSE(icp20100SampleValid);
}

TEST(baroIcp20100Test, AsyncCompletionErrorInvalidatesWithoutBlockingReconfiguration)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 1;
    encodeFrame(fifoData, 100, 200);
    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    simulatedBusError = true;
    simulatedRegisters[0xCD] = 0x00;
    icp20100SampleValid = true;

    EXPECT_TRUE(icp20100GetUP(&simulatedBaro));
    EXPECT_FALSE(icp20100SampleValid);
    EXPECT_EQ(14, icp20100SamplesToDiscard);
    EXPECT_EQ(0U, registerReadCount[0xCD]);
    EXPECT_LT(findRegisterWrite(0xC4, 0x80), 0);
    EXPECT_LT(findRegisterWrite(0xC0, 0x48), 0);
    EXPECT_EQ(1U, dummyReadCount);
}

TEST(baroIcp20100Test, DummyReadFailureRejectsCompletedBurst)
{
    resetSimulatedDevice();
    simulatedRegisters[0xC4] = 1;
    encodeFrame(fifoData, 100, 200);
    ASSERT_TRUE(icp20100ReadUP(&simulatedBaro));
    failDummyRead = true;

    EXPECT_TRUE(icp20100GetUP(&simulatedBaro));
    EXPECT_FALSE(icp20100SampleValid);
}

TEST(baroIcp20100Test, OtpFailureDisablesOtpRelocksAndReturnsStandby)
{
    resetSimulatedDevice();
    const extDevice_t dev = {};
    failReadRegister = 0xB8;

    EXPECT_FALSE(icp20100LoadOtpTrim(&dev, 0x00));
    EXPECT_GE(findRegisterWrite(0xAC, 0x00), 0);
    EXPECT_GE(findRegisterWrite(0xBE, 0x00), 0);
    EXPECT_GE(findRegisterWrite(0xC0, 0x00), 0);
    EXPECT_GE(dummyReadCount, 1U);
}

TEST(baroIcp20100Test, OtpBusyTimeoutRunsFullCleanup)
{
    resetSimulatedDevice();
    const extDevice_t dev = {};
    simulatedRegisters[0xB9] = 0x01;

    EXPECT_FALSE(icp20100LoadOtpTrim(&dev, 0x00));
    EXPECT_GE(findRegisterWrite(0xAC, 0x00), 0);
    EXPECT_GE(findRegisterWrite(0xBE, 0x00), 0);
    EXPECT_GE(findRegisterWrite(0xC0, 0x00), 0);
    EXPECT_GE(dummyReadCount, 1U);
}

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}

bool busBusy(const extDevice_t *, bool *error)
{
    if (error) {
        *error = simulatedBusError;
    }
    return simulatedBusBusy;
}

bool busReadRegisterBuffer(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (length != 1) {
        return false;
    }

    if (reg == failReadRegister || (reg == 0x00 && failDummyRead)) {
        return false;
    }
    registerReadCount[reg]++;
    if (reg == 0x00) {
        dummyReadCount++;
    }

    if (reg == 0xB8) {
        switch (simulatedRegisters[0xB5]) {
        case 0xF8:
            *data = 0x2A;
            break;
        case 0xF9:
            *data = 0x05;
            break;
        case 0xFA:
            *data = 0x55;
            break;
        default:
            return false;
        }
    } else {
        *data = simulatedRegisters[reg];
    }
    return true;
}

bool busReadRegisterBufferStart(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (failAsyncReadStart) {
        return false;
    }
    asyncReadRegister = reg;
    asyncReadLength = length;
    if (reg == 0xFA) {
        memcpy(data, fifoData, length);
    }
    return true;
}

bool busWriteRegister(const extDevice_t *, uint8_t reg, uint8_t value)
{
    if (registerWriteCount < ARRAYLEN(registerWrites)) {
        registerWrites[registerWriteCount++] = { reg, value };
    }
    if (reg == failWriteRegister) {
        return false;
    }
    simulatedRegisters[reg] = value;
    return true;
}

void busDeviceRegister(const extDevice_t *) { registeredDeviceCount++; }

}
