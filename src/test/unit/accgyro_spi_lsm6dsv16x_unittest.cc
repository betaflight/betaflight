/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <array>
#include <cstring>
#include <vector>

extern "C" {
#include "platform.h"
#include "target.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsv16x.h"
#include "drivers/system.h"
#include "drivers/time.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
}

#include "gtest/gtest.h"

namespace {

// Register addresses and expected values from the ST register maps.
constexpr uint8_t INT1_CTRL = 0x0d;
constexpr uint8_t WHO_AM_I = 0x0f;
constexpr uint8_t CTRL1 = 0x10;
constexpr uint8_t CTRL2 = 0x11;
constexpr uint8_t CTRL3 = 0x12;
constexpr uint8_t CTRL6 = 0x15;
constexpr uint8_t CTRL7 = 0x16;
constexpr uint8_t CTRL8 = 0x17;
constexpr uint8_t CTRL9 = 0x18;
constexpr uint8_t OUTX_L_G = 0x22;
constexpr uint8_t OUTX_L_A = 0x28;
constexpr uint8_t HAODR_CFG = 0x62;

enum class Variant { Dsv16x, Dsv32x, Dsk320x };

mpuSensor_e expectedSensor(Variant variant)
{
    switch (variant) {
#ifdef USE_ACCGYRO_LSM6DSV16X
    case Variant::Dsv16x: return LSM6DSV16X_SPI;
#endif
#ifdef USE_ACCGYRO_LSM6DSV32X
    case Variant::Dsv32x: return LSM6DSV32X_SPI;
#endif
#ifdef USE_ACCGYRO_LSM6DSK320X
    case Variant::Dsk320x: return LSM6DSK320X_SPI;
#endif
    default: return MPU_NONE;
    }
}

uint8_t detectSensor(const extDevice_t *dev)
{
#if defined(USE_ACCGYRO_LSM6DSV16X) || defined(USE_ACCGYRO_LSM6DSV32X)
    const uint8_t sensor = lsm6dsv16xSpiDetect(dev);
    if (sensor != MPU_NONE) {
        return sensor;
    }
#endif
#ifdef USE_ACCGYRO_LSM6DSK320X
    return lsm6dsk320xSpiDetect(dev);
#else
    return MPU_NONE;
#endif
}

bool detectGyroCallbacks(gyroDev_t *gyro)
{
#ifdef USE_ACCGYRO_LSM6DSK320X
    if (lsm6dsk320xSpiGyroDetect(gyro)) {
        return true;
    }
#endif
#if defined(USE_ACCGYRO_LSM6DSV16X) || defined(USE_ACCGYRO_LSM6DSV32X)
    return lsm6dsv16xSpiGyroDetect(gyro);
#else
    return false;
#endif
}

bool detectAccCallbacks(accDev_t *acc)
{
#ifdef USE_ACCGYRO_LSM6DSK320X
    if (lsm6dsk320xSpiAccDetect(acc)) {
        return true;
    }
#endif
#if defined(USE_ACCGYRO_LSM6DSV16X) || defined(USE_ACCGYRO_LSM6DSV32X)
    return lsm6dsv16xSpiAccDetect(acc);
#else
    return false;
#endif
}

struct RegisterWrite {
    uint8_t reg;
    uint8_t value;
    timeUs_t atUs;
};

std::array<uint8_t, 128> registers;
std::vector<RegisterWrite> writes;
std::array<int16_t, 6> nextSample;
Variant physicalVariant;
timeMs_t elapsedMs;
timeUs_t elapsedUs;
unsigned resetReads;
unsigned resetBusyReads;
unsigned mpuInitCalls;
unsigned mpuCallbackCalls;
unsigned spiWaitCalls;
int failure;
int droppedWrite;
bool resetPending;
bool useDma;
bool disconnected;

size_t findWrite(uint8_t reg, uint8_t value, size_t start = 0)
{
    for (size_t i = start; i < writes.size(); ++i) {
        if (writes[i].reg == reg && writes[i].value == value) {
            return i;
        }
    }
    return writes.size();
}

class AccgyroSpiLsm6dsv : public ::testing::TestWithParam<Variant> {
protected:
    gyroDev_t gyro;
    accDev_t acc;
    alignas(4) uint8_t txBuffer[32];
    alignas(4) uint8_t rxBuffer[32];

    void SetUp() override
    {
        registers.fill(0);
        physicalVariant = GetParam();
        writes.clear();
        nextSample = {{255, -256, 32767, -32768, 1024, -1024}};
        elapsedMs = resetReads = mpuInitCalls = mpuCallbackCalls = spiWaitCalls = 0;
        elapsedUs = 0;
        resetBusyReads = 2;
        failure = droppedWrite = -1;
        resetPending = disconnected = false;
        useDma = true;
        gyro = {};
        acc = {};
        memset(txBuffer, 0, sizeof(txBuffer));
        memset(rxBuffer, 0, sizeof(rxBuffer));
        gyro.dev.txBuf = txBuffer;
        gyro.dev.rxBuf = rxBuffer;
        gyro.mpuDetectionResult.sensor = expectedSensor(GetParam());
        acc.mpuDetectionResult = gyro.mpuDetectionResult;
        acc.gyro = &gyro;
        registers[WHO_AM_I] = is320x() ? 0x75 : 0x70;
        registers[CTRL8] = is32x() ? 0x04 : 0;
        // An MCU-only restart can leave both sensors running.
        registers[CTRL1] = 0x19;
        registers[CTRL2] = 0x1c;
        gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
        ASSERT_TRUE(detectGyroCallbacks(&gyro));
        ASSERT_TRUE(detectAccCallbacks(&acc));
    }

    bool is320x() const { return GetParam() == Variant::Dsk320x; }
    bool is32x() const { return GetParam() == Variant::Dsv32x; }

    void expectSample(const std::array<int16_t, 6>& sample)
    {
        ASSERT_TRUE(gyro.readFn(&gyro));
        ASSERT_TRUE(acc.readFn(&acc));
        for (unsigned axis = 0; axis < 3; ++axis) {
            EXPECT_EQ(sample[axis], gyro.gyroADCRaw[axis]);
            EXPECT_EQ(sample[axis + 3], acc.ADCRaw[axis]);
        }
    }

    void startDma()
    {
        gyro.detectedEXTI = 1001;
        ASSERT_FALSE(gyro.readFn(&gyro));
        ASSERT_EQ(GYRO_EXTI_INT_DMA, gyro.gyroModeSPI);
        ASSERT_EQ(13, gyro.segments[0].len);
        ASSERT_NE(nullptr, gyro.segments[0].callback);
        ASSERT_EQ(0, gyro.segments[1].len);
    }

    void completeDma()
    {
        // The command byte is followed by six little-endian sensor words.
        uint8_t *data = gyro.segments[0].u.buffers.rxData;
        data[0] = 0;
        memcpy(data + 1, nextSample.data(), sizeof(nextSample));
        ASSERT_EQ(BUS_READY, gyro.segments[0].callback(gyro.dev.callbackArg));
    }
};

TEST_P(AccgyroSpiLsm6dsv, DetectsExpectedIdentity)
{
    EXPECT_EQ(gyro.mpuDetectionResult.sensor, detectSensor(&gyro.dev));
    registers[WHO_AM_I] = 0xff;
    EXPECT_EQ(MPU_NONE, detectSensor(&gyro.dev));
}

TEST_P(AccgyroSpiLsm6dsv, DetectsOnlyEnabledVariants)
{
    for (const Variant variant : {Variant::Dsv16x, Variant::Dsv32x, Variant::Dsk320x}) {
        physicalVariant = variant;
        registers[WHO_AM_I] = variant == Variant::Dsk320x ? 0x75 : 0x70;
        registers[CTRL8] = variant == Variant::Dsv32x ? 0x04 : 0;
        resetReads = 0;
        EXPECT_EQ(expectedSensor(variant), detectSensor(&gyro.dev));
    }
}

TEST_P(AccgyroSpiLsm6dsv, CallbackDetectionRejectsDisabledVariants)
{
    for (const mpuSensor_e sensor : {LSM6DSV16X_SPI, LSM6DSV32X_SPI, LSM6DSK320X_SPI, MPU_NONE}) {
        gyro.mpuDetectionResult.sensor = sensor;
        acc.mpuDetectionResult.sensor = sensor;
        const bool enabled = sensor != MPU_NONE &&
            (sensor == expectedSensor(Variant::Dsv16x) ||
             sensor == expectedSensor(Variant::Dsv32x) ||
             sensor == expectedSensor(Variant::Dsk320x));
        EXPECT_EQ(enabled, detectGyroCallbacks(&gyro));
        EXPECT_EQ(enabled, detectAccCallbacks(&acc));
    }
}

#if defined(USE_ACCGYRO_LSM6DSV16X) || defined(USE_ACCGYRO_LSM6DSV32X)
TEST_P(AccgyroSpiLsm6dsv, DetectsVariantFromResetDefaults)
{
    if (is320x()) {
        return;
    }
    // Firmware running before an MCU-only restart may have changed CTRL8.
    registers[CTRL8] = is32x() ? 0 : 0x04;
    EXPECT_EQ(gyro.mpuDetectionResult.sensor, detectSensor(&gyro.dev));
    EXPECT_LT(findWrite(CTRL3, 0x01), writes.size());
    EXPECT_EQ(is32x() ? 0x04 : 0, registers[CTRL8]);
}

TEST_P(AccgyroSpiLsm6dsv, ResetTimeoutRejectsDetection)
{
    if (is320x()) {
        return;
    }
    resetBusyReads = 1000;
    EXPECT_EQ(MPU_NONE, detectSensor(&gyro.dev));
    EXPECT_LT(resetReads, 100u);
    EXPECT_LE(elapsedMs, 100u);
}
#endif

TEST_P(AccgyroSpiLsm6dsv, ConfiguresHighAccuracyRatesFiltersAndFullScale)
{
    gyro.initFn(&gyro);
    ASSERT_EQ(-1, failure);
    EXPECT_EQ(1u, mpuInitCalls);
    EXPECT_EQ(0x19, registers[CTRL1]); // 1 kHz accel, high accuracy mode
    EXPECT_EQ(0x1c, registers[CTRL2]); // 8 kHz gyro, high accuracy mode
    EXPECT_EQ(0x01, registers[HAODR_CFG]);
    EXPECT_EQ(0x44, registers[CTRL3]); // Block-data update and address increment
    EXPECT_EQ(is32x() ? 0x0c : 0x04, registers[CTRL6]);
    EXPECT_EQ(0x01, registers[CTRL7]); // Gyro LPF1
    EXPECT_EQ(is32x() ? 0x07 : 0x03, registers[CTRL8]); // ODR/4, variant and FSR
    EXPECT_EQ(0x08, registers[CTRL9]); // Accel LPF2 enable
    EXPECT_EQ(0x02, registers[INT1_CTRL]);
    EXPECT_FLOAT_EQ(is32x() ? 0.140f : 0.070f, gyro.scale);
    acc.initFn(&acc);
    EXPECT_EQ(is32x() ? 1024 : 2048, acc.acc_1G);
}

TEST_P(AccgyroSpiLsm6dsv, PowersDownBeforeResetAndStartsGyroBeforeAccelerometer)
{
    gyro.initFn(&gyro);
    ASSERT_EQ(-1, failure);
    const size_t reset = findWrite(CTRL3, 0x01);
    ASSERT_LT(reset, writes.size());
    const size_t accPowerDown = findWrite(CTRL1, 0x10);
    const size_t gyroPowerDown = findWrite(CTRL2, 0x10);
    ASSERT_LT(accPowerDown, reset);
    ASSERT_LT(gyroPowerDown, reset);
    EXPECT_GE(writes[reset].atUs - writes[accPowerDown].atUs, 500u);
    EXPECT_GE(writes[reset].atUs - writes[gyroPowerDown].atUs, 500u);

    const size_t gyroStart = findWrite(CTRL2, 0x1c, reset + 1);
    const size_t accStart = findWrite(CTRL1, 0x19, reset + 1);
    ASSERT_LT(gyroStart, writes.size());
    ASSERT_LT(accStart, writes.size());
    const size_t accMode = findWrite(CTRL1, 0x10, reset + 1);
    const size_t gyroMode = findWrite(CTRL2, 0x10, reset + 1);
    ASSERT_LT(accMode, gyroStart);
    ASSERT_LT(gyroMode, gyroStart);
    EXPECT_GE(writes[gyroStart].atUs - writes[accMode].atUs, 500u);
    EXPECT_GE(writes[gyroStart].atUs - writes[gyroMode].atUs, 500u);
    EXPECT_LT(gyroStart, accStart);
}

TEST_P(AccgyroSpiLsm6dsv, AppliesSelectedGyroLpfWithoutChangingRange)
{
    gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_OPTION_1;
    gyro.initFn(&gyro);
    ASSERT_EQ(-1, failure);
    EXPECT_EQ(is32x() ? 0x2c : 0x24, registers[CTRL6]);
}

TEST_P(AccgyroSpiLsm6dsv, ResetTimeoutFailsBeforeEnablingInterrupts)
{
    // The guard keeps an accidentally unbounded polling implementation from
    // hanging the test executable; a correct timeout returns well before it.
    resetBusyReads = 1000;
    gyro.initFn(&gyro);
    EXPECT_EQ(FAILURE_GYRO_INIT_FAILED, failure);
    EXPECT_EQ(0u, mpuInitCalls);
    EXPECT_LT(resetReads, 100u);
    EXPECT_LT(elapsedMs, 100u);
    EXPECT_EQ(writes.size(), findWrite(INT1_CTRL, 0x02));
}

TEST_P(AccgyroSpiLsm6dsv, DisconnectedBusFailsInitialization)
{
    disconnected = true;
    gyro.initFn(&gyro);
    EXPECT_EQ(FAILURE_GYRO_INIT_FAILED, failure);
    EXPECT_EQ(0u, mpuInitCalls);
    EXPECT_LT(resetReads, 100u);
    EXPECT_EQ(writes.size(), findWrite(INT1_CTRL, 0x02));
}

TEST_P(AccgyroSpiLsm6dsv, FailedCriticalRegisterWriteFailsInitialization)
{
    droppedWrite = CTRL9;
    gyro.initFn(&gyro);
    EXPECT_EQ(FAILURE_GYRO_INIT_FAILED, failure);
    EXPECT_EQ(0u, mpuInitCalls);
}

TEST_P(AccgyroSpiLsm6dsv, DmaReadersRejectDataBeforeFirstCompletedTransfer)
{
    EXPECT_FALSE(acc.readFn(&acc));
    startDma();
    memset(rxBuffer, 0x55, sizeof(rxBuffer));
    EXPECT_FALSE(gyro.readFn(&gyro));
    EXPECT_FALSE(acc.readFn(&acc));
    EXPECT_EQ(0u, mpuCallbackCalls);
    completeDma();
    expectSample(nextSample);
}

TEST_P(AccgyroSpiLsm6dsv, DmaReadersPreserveRawAxesDuringPublication)
{
    startDma();
    completeDma();
    const auto previous = nextSample;
    expectSample(previous);

    ++gyro.lsm6dsvDmaSequence; // Odd means the completion ISR is publishing.
    memset(gyro.lsm6dsvDmaSample, 0x55, sizeof(gyro.lsm6dsvDmaSample));
    EXPECT_FALSE(gyro.readFn(&gyro));
    EXPECT_FALSE(acc.readFn(&acc));
    for (unsigned axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(previous[axis], gyro.gyroADCRaw[axis]);
        EXPECT_EQ(previous[axis + 3], acc.ADCRaw[axis]);
    }
}

TEST_P(AccgyroSpiLsm6dsv, DmaPublicationSequenceCanWrapToZero)
{
    startDma();
    gyro.lsm6dsvDmaSequence = UINT32_MAX - 1;
    completeDma();
    EXPECT_EQ(0u, gyro.lsm6dsvDmaSequence);
    expectSample(nextSample);
}

TEST_P(AccgyroSpiLsm6dsv, DmaReadersKeepCompletedSampleDuringNextTransfer)
{
    startDma();
    completeDma();
    ASSERT_EQ(1u, mpuCallbackCalls);
    const auto previous = nextSample;
    expectSample(previous);

    // The next X value is 0x0100; after just its low byte arrives the DMA
    // buffer contains 0x0000. Readers must still return completed 0x00ff.
    uint8_t *data = gyro.segments[0].u.buffers.rxData;
    data[1] = 0;
    data[7] = 0x55; // Partially update an accelerometer word as well.
    expectSample(previous);
    EXPECT_EQ(0u, spiWaitCalls);

    nextSample = {{256, 1234, -1234, 32767, -32768, 256}};
    completeDma();
    ASSERT_EQ(2u, mpuCallbackCalls);
    expectSample(nextSample);
}

TEST_P(AccgyroSpiLsm6dsv, DmaCompletedSamplesBelongToTheirOwnDevice)
{
    startDma();
    completeDma();
    const auto firstSample = nextSample;

    gyroDev_t otherGyro = gyro;
    accDev_t otherAcc = acc;
    alignas(4) uint8_t otherTx[32] = {};
    alignas(4) uint8_t otherRx[32] = {};
    otherGyro.dev.txBuf = otherTx;
    otherGyro.dev.rxBuf = otherRx;
    otherGyro.gyroModeSPI = GYRO_EXTI_INIT;
    otherAcc.gyro = &otherGyro;
    ASSERT_FALSE(otherGyro.readFn(&otherGyro));
    ASSERT_EQ(GYRO_EXTI_INT_DMA, otherGyro.gyroModeSPI);

    const std::array<int16_t, 6> secondSample = {{-55, 66, -77, 88, -99, 111}};
    memcpy(otherGyro.segments[0].u.buffers.rxData + 1, secondSample.data(), sizeof(secondSample));
    ASSERT_EQ(BUS_READY, otherGyro.segments[0].callback(otherGyro.dev.callbackArg));
    ASSERT_TRUE(otherGyro.readFn(&otherGyro));
    ASSERT_TRUE(otherAcc.readFn(&otherAcc));
    for (unsigned axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(secondSample[axis], otherGyro.gyroADCRaw[axis]);
        EXPECT_EQ(secondSample[axis + 3], otherAcc.ADCRaw[axis]);
    }
    expectSample(firstSample);
}

TEST_P(AccgyroSpiLsm6dsv, BlockingReadsWithoutInterruptsKeepAxisOffsets)
{
    ASSERT_FALSE(gyro.readFn(&gyro));
    ASSERT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
    expectSample(nextSample);
    EXPECT_EQ(2u, spiWaitCalls);
}

TEST_P(AccgyroSpiLsm6dsv, BlockingReadsWithInterruptsWithoutDmaKeepAxisOffsets)
{
    useDma = false;
    gyro.detectedEXTI = 1001;
    ASSERT_FALSE(gyro.readFn(&gyro));
    ASSERT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
    expectSample(nextSample);
    EXPECT_EQ(2u, spiWaitCalls);
}

INSTANTIATE_TEST_SUITE_P(Variants, AccgyroSpiLsm6dsv,
    ::testing::ValuesIn(std::vector<Variant>{
#ifdef USE_ACCGYRO_LSM6DSV16X
        Variant::Dsv16x,
#endif
#ifdef USE_ACCGYRO_LSM6DSV32X
        Variant::Dsv32x,
#endif
#ifdef USE_ACCGYRO_LSM6DSK320X
        Variant::Dsk320x,
#endif
    }));

} // namespace

extern "C" {

void delay(timeMs_t ms)
{
    elapsedMs += ms;
    elapsedUs += ms * 1000;
}
void delayMicroseconds(timeUs_t us) { elapsedUs += us; }
timeMs_t millis(void) { return elapsedMs; }
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
uint16_t spiCalculateDivider(uint32_t) { return 2; }

void spiWriteReg(const extDevice_t *, uint8_t reg, uint8_t value)
{
    writes.push_back({reg, value, elapsedUs});
    if (reg == droppedWrite) {
        return;
    }
    if (reg == CTRL3 && (value & 0x01)) {
        const uint8_t identity = registers[WHO_AM_I];
        const uint8_t variant = physicalVariant == Variant::Dsv32x ? 0x04 : 0;
        registers.fill(0);
        registers[WHO_AM_I] = identity;
        registers[CTRL8] = variant;
        registers[CTRL3] = 0x04;
        resetPending = true;
    } else {
        registers[reg] = value;
    }
}

uint8_t spiReadRegMsk(const extDevice_t *, uint8_t reg)
{
    if (resetPending && reg == CTRL3) {
        ++resetReads;
        if (resetReads <= resetBusyReads || (disconnected && resetReads < 1000)) {
            return disconnected ? 0xff : 0x01;
        }
        resetPending = false;
    }
    return disconnected ? (reg == CTRL3 ? 0 : 0xff) : registers[reg];
}

void mpuGyroInit(gyroDev_t *) { ++mpuInitCalls; }
void failureMode(failureMode_e mode) { failure = mode; }
bool spiUseDMA(const extDevice_t *) { return useDma; }
void spiWait(const extDevice_t *) { ++spiWaitCalls; }

void spiSequence(const extDevice_t *, busSegment_t *segments)
{
    const uint8_t address = segments[0].u.buffers.txData[0] & 0x7f;
    EXPECT_TRUE(address == OUTX_L_G || address == OUTX_L_A);
    EXPECT_EQ(7, segments[0].len);
    uint8_t *data = segments[0].u.buffers.rxData;
    data[0] = 0;
    memcpy(data + 1, nextSample.data() + (address == OUTX_L_A ? 3 : 0), 6);
}

busStatus_e mpuIntCallback(uintptr_t arg)
{
    ++mpuCallbackCalls;
    reinterpret_cast<gyroDev_t *>(arg)->dataReady = true;
    return BUS_READY;
}

} // extern "C"
