/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 */

#include <stdint.h>

extern "C" {

#include "platform.h"
#include "target.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_bmi088.h"
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/io.h"

STATIC_UNIT_TESTED uint16_t bmi088AccOneG(bool highFsr);
STATIC_UNIT_TESTED uint8_t bmi088AccRange(bool highFsr);
STATIC_UNIT_TESTED uint8_t bmi088GyroBandwidth(uint8_t hardwareLpf);
STATIC_UNIT_TESTED void bmi088DecodeLittleEndian(const uint8_t raw[6], int16_t data[XYZ_AXIS_COUNT]);

}

#include "gtest/gtest.h"

static uint8_t mockedGyroChipId;
static unsigned accelReadCount;
static unsigned registeredDeviceCount;
static busDevice_t mockedSpiBus;
static bool returnAccelData;
static bool returnGyroData;
static uint8_t lastAccelReadReg;
static uint8_t lastAccelReadLength;
static uint32_t elapsedUs;

typedef struct writeRecord_s {
    uint8_t reg;
    uint8_t value;
    uint32_t timestampUs;
} writeRecord_t;

static writeRecord_t writeRecords[16];
static unsigned writeRecordCount;

static void resetBusMocks(void)
{
    accelReadCount = 0;
    registeredDeviceCount = 0;
    returnAccelData = false;
    returnGyroData = false;
    lastAccelReadReg = 0;
    lastAccelReadLength = 0;
    elapsedUs = 0;
    writeRecordCount = 0;
}

static void configureBmi088Accelerometer(gyroDev_t *gyro, accDev_t *acc)
{
    gyro->mpuDetectionResult.sensor = BMI_088_SPI;
    gyro->accCsnTag = 42;
    gyro->spiBus = 2;
    gyro->deviceIndex = 1;

    acc->gyro = gyro;
    acc->mpuDetectionResult.sensor = BMI_088_SPI;
}

TEST(AccgyroSpiBmi088, DecodesLittleEndianSignedAxes)
{
    const uint8_t raw[6] = { 0x34, 0x12, 0x00, 0x80, 0xff, 0x7f };
    int16_t data[XYZ_AXIS_COUNT] = {};

    bmi088DecodeLittleEndian(raw, data);

    EXPECT_EQ(0x1234, data[X]);
    EXPECT_EQ(INT16_MIN, data[Y]);
    EXPECT_EQ(INT16_MAX, data[Z]);
}

TEST(AccgyroSpiBmi088, SelectsDocumentedAccelerometerRanges)
{
    EXPECT_EQ(0x02, bmi088AccRange(false));
    EXPECT_EQ(2730, bmi088AccOneG(false));
    EXPECT_EQ(0x03, bmi088AccRange(true));
    EXPECT_EQ(1365, bmi088AccOneG(true));
}

TEST(AccgyroSpiBmi088, SelectsDocumentedGyroscopeBandwidths)
{
    // All choices keep both the documented reset value in bit 7 and a 2 kHz
    // ODR in bits 3:0 to match gyroSetSampleRate().
    EXPECT_EQ(0x81, bmi088GyroBandwidth(GYRO_HARDWARE_LPF_NORMAL));
    EXPECT_EQ(0x81, bmi088GyroBandwidth(GYRO_HARDWARE_LPF_OPTION_1));
    EXPECT_EQ(0x80, bmi088GyroBandwidth(GYRO_HARDWARE_LPF_OPTION_2));
}

TEST(AccgyroSpiBmi088, UsesTwoKilohertzGyroAndSixteenHundredHertzAccelerometer)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = BMI_088_SPI;

    EXPECT_EQ(2000, gyroSetSampleRate(&gyro));
    EXPECT_EQ(GYRO_RATE_2_kHz, gyro.gyroRateKHz);
    EXPECT_EQ(1600, gyro.accSampleRateHz);
}

TEST(AccgyroSpiBmi088, RejectsWrongMpuDetectionResult)
{
    gyroDev_t gyro = {};
    accDev_t acc = {};

    EXPECT_FALSE(bmi088SpiGyroDetect(&gyro));
    EXPECT_FALSE(bmi088SpiAccDetect(&acc));
}

TEST(AccgyroSpiBmi088, DetectsOfficialGyroscopeChipId)
{
    extDevice_t dev = {};
    mockedGyroChipId = BMI088_GYRO_CHIP_ID;
    EXPECT_EQ(BMI_088_SPI, bmi088GyroDetect(&dev));

    mockedGyroChipId = 0;
    EXPECT_EQ(MPU_NONE, bmi088GyroDetect(&dev));
}

TEST(AccgyroSpiBmi088, AllowsGyroscopeWithoutAccelerometerChipSelect)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = BMI_088_SPI;

    EXPECT_TRUE(bmi088SpiGyroDetect(&gyro));

    accDev_t acc = {};
    acc.gyro = &gyro;
    acc.mpuDetectionResult.sensor = BMI_088_SPI;
    EXPECT_FALSE(bmi088SpiAccDetect(&acc));
}

TEST(AccgyroSpiBmi088, ConfiguresIndependentAccelerometerDevice)
{
    gyroDev_t gyro = {};
    accDev_t acc = {};
    configureBmi088Accelerometer(&gyro, &acc);

    resetBusMocks();
    EXPECT_TRUE(bmi088SpiAccDetect(&acc));
    EXPECT_EQ(2u, accelReadCount); // first read selects SPI, second verifies ID
    EXPECT_EQ(1u, registeredDeviceCount);
    EXPECT_EQ(reinterpret_cast<IO_t>(static_cast<uintptr_t>(gyro.accCsnTag)), acc.dev.busType_u.spi.csnPin);
    EXPECT_NE(&gyro.dev, &acc.dev);
}

TEST(AccgyroSpiBmi088, SkipsAccelerometerSpiDummyByte)
{
    gyroDev_t gyro = {};
    accDev_t acc = {};
    configureBmi088Accelerometer(&gyro, &acc);
    resetBusMocks();
    ASSERT_TRUE(bmi088SpiAccDetect(&acc));

    returnAccelData = true;
    ASSERT_TRUE(acc.readFn(&acc));

    EXPECT_EQ(0x92, lastAccelReadReg);
    EXPECT_EQ(7, lastAccelReadLength);
    EXPECT_EQ(0x1234, acc.ADCRaw[X]);
    EXPECT_EQ(INT16_MIN, acc.ADCRaw[Y]);
    EXPECT_EQ(INT16_MAX, acc.ADCRaw[Z]);
}

TEST(AccgyroSpiBmi088, ObservesSuspendWriteDelayAndPowerUpDelay)
{
    gyroDev_t gyro = {};
    accDev_t acc = {};
    configureBmi088Accelerometer(&gyro, &acc);
    resetBusMocks();
    ASSERT_TRUE(bmi088SpiAccDetect(&acc));

    elapsedUs = 0;
    writeRecordCount = 0;
    acc.initFn(&acc);

    ASSERT_EQ(5u, writeRecordCount);
    EXPECT_EQ(0x7e, writeRecords[0].reg);
    EXPECT_EQ(0xb6, writeRecords[0].value);
    EXPECT_EQ(0x7c, writeRecords[1].reg);
    EXPECT_EQ(0x00, writeRecords[1].value);
    EXPECT_GE(writeRecords[2].timestampUs - writeRecords[1].timestampUs, 1000u);
    EXPECT_EQ(0x7d, writeRecords[2].reg);
    EXPECT_EQ(0x04, writeRecords[2].value);
    EXPECT_GE(writeRecords[3].timestampUs - writeRecords[2].timestampUs, 450u);
    EXPECT_EQ(0x40, writeRecords[3].reg);
    EXPECT_EQ(0x9c, writeRecords[3].value);
    EXPECT_EQ(0x41, writeRecords[4].reg);
    EXPECT_EQ(0x02, writeRecords[4].value);
    EXPECT_GE(writeRecords[4].timestampUs - writeRecords[3].timestampUs, 2u);
}

TEST(AccgyroSpiBmi088, UsesExtiOnlyAfterSustainedStartupPulses)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = BMI_088_SPI;
    gyro.mpuIntExtiTag = 43;
    ASSERT_TRUE(bmi088SpiGyroDetect(&gyro));

    resetBusMocks();
    gyro.initFn(&gyro);
    EXPECT_EQ(GYRO_EXTI_INIT, gyro.gyroModeSPI);

    gyro.detectedEXTI = 1001;
    returnGyroData = true;
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}

TEST(AccgyroSpiBmi088, FallsBackToPollingWithoutStartupPulses)
{
    gyroDev_t gyro = {};
    gyro.mpuDetectionResult.sensor = BMI_088_SPI;
    gyro.mpuIntExtiTag = 43;
    ASSERT_TRUE(bmi088SpiGyroDetect(&gyro));

    resetBusMocks();
    gyro.initFn(&gyro);
    EXPECT_EQ(GYRO_EXTI_INIT, gyro.gyroModeSPI);

    returnGyroData = true;
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
}

extern "C" {

void delay(uint32_t milliseconds) { elapsedUs += milliseconds * 1000; }
void delayMicroseconds(uint32_t microseconds) { elapsedUs += microseconds; }
uint32_t getCycleCounter(void) { return 0; }
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
uint16_t spiCalculateDivider(uint32_t) { return 2; }
uint8_t spiReadRegMsk(const extDevice_t *, uint8_t) { return mockedGyroChipId; }
void spiReadRegBuf(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    lastAccelReadReg = reg;
    lastAccelReadLength = length;
    if (length == 2) {
        data[0] = 0xff; // BMI088 accelerometer's mandatory dummy byte
        data[1] = accelReadCount++ ? BMI088_ACC_CHIP_ID : 0x00;
    } else if (length == 7 && returnAccelData) {
        static const uint8_t raw[7] = { 0xee, 0x34, 0x12, 0x00, 0x80, 0xff, 0x7f };
        for (unsigned i = 0; i < sizeof(raw); i++) {
            data[i] = raw[i];
        }
    }
}
bool spiReadRegMskBufRB(const extDevice_t *, uint8_t, uint8_t *data, uint8_t length)
{
    if (!returnGyroData || length != 6) {
        return false;
    }

    static const uint8_t raw[6] = { 0x34, 0x12, 0x00, 0x80, 0xff, 0x7f };
    for (unsigned i = 0; i < sizeof(raw); i++) {
        data[i] = raw[i];
    }
    return true;
}
bool spiIsBusy(const extDevice_t *) { return false; }
void spiWriteReg(const extDevice_t *, uint8_t reg, uint8_t value)
{
    if (writeRecordCount < sizeof(writeRecords) / sizeof(writeRecords[0])) {
        writeRecords[writeRecordCount++] = { reg, value, elapsedUs };
    }
}
bool spiSetBusInstance(extDevice_t *dev, uint32_t)
{
    dev->bus = &mockedSpiBus;
    return true;
}
void busDeviceRegister(const extDevice_t *) { registeredDeviceCount++; }
IO_t IOGetByTag(ioTag_t tag) { return reinterpret_cast<IO_t>(static_cast<uintptr_t>(tag)); }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOHi(IO_t) {}
void ioPreinitByIO(IO_t, uint8_t, ioPreinitPinState_e) {}
void EXTIHandlerInit(extiCallbackRec_t *, extiHandlerCallback *) {}
void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t) {}
void EXTIEnable(IO_t) {}

}
