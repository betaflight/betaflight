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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <string.h>
#include <vector>

extern "C" {

#include "platform.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_adis1657x.h"
#include "drivers/bus.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

} // extern "C"

#include "unittest_macros.h"
#include "gtest/gtest.h"

// A fake ADIS1657x behind the SPI stubs at the bottom of the file, so the
// driver's real init and read paths run unmodified. Register access is modelled
// as the part does it: byte-addressed halves, and reads pipelined one frame late.
struct FakeAdis {
    uint16_t reg[0x80];
    std::vector<std::pair<uint8_t, uint16_t>> writes;
    uint16_t burstFrame[ADIS1657X_BURST_WORD_COUNT]; // wire order, big-endian
    int burstCount;
    uint16_t pending;       // answer latched by the address frame
    uint8_t pendingLowByte; // low half of a write waiting for its high half
    bool pendingLowValid;
    uint8_t pendingLowReg;

    void reset()
    {
        memset(reg, 0, sizeof(reg));
        reg[ADIS1657X_REG_PROD_ID] = ADIS1657X_PROD_ID_16575;
        reg[ADIS1657X_REG_RNG_MDL] = ADIS1657X_RNG_MDL_LOW_RANGE;
        writes.clear();
        memset(burstFrame, 0, sizeof(burstFrame));
        burstCount = 0;
        pending = 0;
        pendingLowByte = 0;
        pendingLowValid = false;
        pendingLowReg = 0;
    }

    bool wrote(uint8_t r, uint16_t v) const
    {
        for (const auto &w : writes) {
            if (w.first == r && w.second == v) {
                return true;
            }
        }
        return false;
    }

    bool wroteReg(uint8_t r) const
    {
        for (const auto &w : writes) {
            if (w.first == r) {
                return true;
            }
        }
        return false;
    }
};

static FakeAdis fake;

/*
 * Build a burst frame in the device's wire format: big-endian 32-bit axes as a
 * low word then a high word, and a trailing byte-wise sum over the words from
 * the first gyro word up to and including the timestamp.
 */
static void fakeSetBurst(int16_t gx, int16_t gy, int16_t gz,
                         int16_t ax, int16_t ay, int16_t az,
                         int16_t temp, uint16_t dataCntr)
{
    uint16_t host[ADIS1657X_BURST_WORD_COUNT] = {};

    host[ADIS1657X_BURST_WORD_FIFO_CNT] = 0;
    host[ADIS1657X_BURST_WORD_DIAG_STAT] = 0;
    host[ADIS1657X_BURST_GYRO_HIGH(0)] = (uint16_t)gx;
    host[ADIS1657X_BURST_GYRO_HIGH(1)] = (uint16_t)gy;
    host[ADIS1657X_BURST_GYRO_HIGH(2)] = (uint16_t)gz;
    host[ADIS1657X_BURST_ACCEL_HIGH(0)] = (uint16_t)ax;
    host[ADIS1657X_BURST_ACCEL_HIGH(1)] = (uint16_t)ay;
    host[ADIS1657X_BURST_ACCEL_HIGH(2)] = (uint16_t)az;
    host[ADIS1657X_BURST_WORD_TEMPERATURE] = (uint16_t)temp;
    host[ADIS1657X_BURST_WORD_DATA_CNTR] = dataCntr;
    host[ADIS1657X_BURST_WORD_TIMESTAMP] = 0;

    for (int i = 0; i < ADIS1657X_BURST_WORD_COUNT; i++) {
        fake.burstFrame[i] = __builtin_bswap16(host[i]);
    }

    const uint8_t *bytes = (const uint8_t *)&fake.burstFrame[ADIS1657X_BURST_CHECKSUM_FIRST_WORD];
    const uint8_t *end = (const uint8_t *)&fake.burstFrame[ADIS1657X_BURST_CHECKSUM_END_WORD];
    uint16_t sum = 0;
    while (bytes < end) {
        sum += *bytes++;
    }
    fake.burstFrame[ADIS1657X_BURST_WORD_CHECKSUM] = __builtin_bswap16(sum);
}

class Adis1657xTest : public ::testing::Test {
protected:
    gyroDev_t gyro;
    accDev_t acc;
    uint8_t txBuf[ADIS1657X_BURST_FRAME_LEN];
    uint8_t rxBuf[ADIS1657X_BURST_FRAME_LEN];
    static uint16_t cntr;

    void SetUp() override
    {
        fake.reset();
        memset(&gyro, 0, sizeof(gyro));
        memset(&acc, 0, sizeof(acc));
        memset(txBuf, 0, sizeof(txBuf));
        memset(rxBuf, 0, sizeof(rxBuf));
        gyro.dev.txBuf = txBuf;
        gyro.dev.rxBuf = rxBuf;
        gyro.mpuDetectionResult.sensor = ADIS1657X_SPI;
        acc.mpuDetectionResult.sensor = ADIS1657X_SPI;
        acc.gyro = &gyro;
    }

    // The driver tracks the sample counter across calls, so every frame a test
    // expects to be accepted must carry a value no earlier test has used.
    uint16_t nextCntr() { return ++cntr; }

    void initGyro()
    {
        ASSERT_TRUE(adis1657xSpiGyroDetect(&gyro));
        gyro.initFn(&gyro);
    }

    void deliverBurst()
    {
        memcpy(rxBuf, fake.burstFrame, sizeof(fake.burstFrame));
    }
};

uint16_t Adis1657xTest::cntr = 100;

// --- detection ---------------------------------------------------------------

TEST_F(Adis1657xTest, DetectMatchesEachProductId)
{
    extDevice_t dev = {};

    const uint16_t ids[] = {
        ADIS1657X_PROD_ID_16575,
        ADIS1657X_PROD_ID_16576,
        ADIS1657X_PROD_ID_16577,
    };

    for (uint16_t id : ids) {
        fake.reset();
        fake.reg[ADIS1657X_REG_PROD_ID] = id;
        EXPECT_EQ(ADIS1657X_SPI, adis1657xSpiDetect(&dev));
    }
}

TEST_F(Adis1657xTest, DetectRejectsUnknownProductId)
{
    extDevice_t dev = {};
    fake.reg[ADIS1657X_REG_PROD_ID] = 0x40BE;
    EXPECT_EQ(MPU_NONE, adis1657xSpiDetect(&dev));
}

TEST_F(Adis1657xTest, DetectPerformsNoWrites)
{
    extDevice_t dev = {};
    adis1657xSpiDetect(&dev);
    EXPECT_TRUE(fake.writes.empty());
}

TEST_F(Adis1657xTest, GyroDetectRejectsOtherSensors)
{
    gyro.mpuDetectionResult.sensor = ICM_42688P_SPI;
    EXPECT_FALSE(adis1657xSpiGyroDetect(&gyro));
}

TEST_F(Adis1657xTest, AccDetectRejectsOtherSensors)
{
    acc.mpuDetectionResult.sensor = ICM_42688P_SPI;
    EXPECT_FALSE(adis1657xSpiAccDetect(&acc));
}

// --- initialisation ----------------------------------------------------------

TEST_F(Adis1657xTest, InitIssuesSoftwareReset)
{
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_GLOB_CMD, ADIS1657X_GLOB_CMD_SW_RESET));
}

TEST_F(Adis1657xTest, InitConfiguresMscCtrlForA32BitRateBurst)
{
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_MSC_CTRL, ADIS1657X_MSC_CTRL_CFG));
    EXPECT_EQ(0, ADIS1657X_MSC_CTRL_CFG & ADIS1657X_MSC_CTRL_OUT_SEL);
    EXPECT_TRUE(ADIS1657X_MSC_CTRL_CFG & ADIS1657X_MSC_CTRL_BURST_32);
    // Data ready active high is the rising edge mpuIntExtiInit() already arms.
    EXPECT_TRUE(ADIS1657X_MSC_CTRL_CFG & ADIS1657X_MSC_CTRL_DR_POL);
    EXPECT_TRUE(ADIS1657X_MSC_CTRL_CFG & ADIS1657X_MSC_CTRL_POP_EN);
    EXPECT_EQ(0, ADIS1657X_MSC_CTRL_CFG & ADIS1657X_MSC_CTRL_GSEN_EN);
}

TEST_F(Adis1657xTest, InitRunsAtFullOutputRateWithFifoDisabled)
{
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_DEC_RATE, ADIS1657X_DEC_RATE_2000HZ));
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_FIFO_CTRL, ADIS1657X_FIFO_CTRL_DISABLED));
}

TEST_F(Adis1657xTest, InitLeavesHardwareFilterOffByDefault)
{
    gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_FILT_CTRL, ADIS1657X_FILT_CTRL_OFF));
}

TEST_F(Adis1657xTest, InitSelectsRequestedHardwareFilter)
{
    gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_OPTION_2;
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS1657X_REG_FILT_CTRL, ADIS1657X_FILT_CTRL_16_TAP));
    gyroConfigMutable()->gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
}

TEST_F(Adis1657xTest, InitTakesGyroScaleFromRangeModelRegister)
{
    fake.reg[ADIS1657X_REG_RNG_MDL] = ADIS1657X_RNG_MDL_LOW_RANGE;
    initGyro();
    EXPECT_FLOAT_EQ(ADIS1657X_GYRO_SCALE_LOW, gyro.scale);

    SetUp();
    fake.reg[ADIS1657X_REG_RNG_MDL] = ADIS1657X_RNG_MDL_HIGH_RANGE;
    initGyro();
    EXPECT_FLOAT_EQ(ADIS1657X_GYRO_SCALE_HIGH, gyro.scale);
}

TEST_F(Adis1657xTest, InitIgnoresTheUnusedRangeModelBits)
{
    fake.reg[ADIS1657X_REG_RNG_MDL] = 0xFFF0 | ADIS1657X_RNG_MDL_HIGH_RANGE;
    initGyro();
    EXPECT_FLOAT_EQ(ADIS1657X_GYRO_SCALE_HIGH, gyro.scale);
}

// Datasheet tables 20 and 35: the numbers a wrong scale would silently mistune.
TEST_F(Adis1657xTest, SensitivitiesMatchTheDatasheet)
{
    EXPECT_FLOAT_EQ(1.0f / 40.0f, ADIS1657X_GYRO_SCALE_LOW);
    EXPECT_FLOAT_EQ(1.0f / 10.0f, ADIS1657X_GYRO_SCALE_HIGH);
    EXPECT_EQ(4000, ADIS1657X_ACC_1G_16575);
    EXPECT_EQ(800, ADIS1657X_ACC_1G_16576);
    EXPECT_EQ(800, ADIS1657X_ACC_1G_16577);
}

TEST_F(Adis1657xTest, InitFallsBackToTheLowRangeScale)
{
    fake.reg[ADIS1657X_REG_RNG_MDL] = 0x0000;
    initGyro();
    EXPECT_FLOAT_EQ(ADIS1657X_GYRO_SCALE_LOW, gyro.scale);
}

TEST_F(Adis1657xTest, InitPointsDmaAtTheBurstCommand)
{
    initGyro();
    EXPECT_EQ(ADIS1657X_BURST_CMD, gyro.dmaReadRegStart);
}

TEST_F(Adis1657xTest, InitPublishesTemperatureConversion)
{
    initGyro();
    EXPECT_FLOAT_EQ(ADIS1657X_TEMP_SCALE, gyro.tempScale);
    EXPECT_FLOAT_EQ(ADIS1657X_TEMP_ZERO, gyro.tempZero);
}

TEST_F(Adis1657xTest, AccSensitivityFollowsTheDetectedVariant)
{
    extDevice_t dev = {};

    fake.reg[ADIS1657X_REG_PROD_ID] = ADIS1657X_PROD_ID_16577;
    adis1657xSpiDetect(&dev);
    ASSERT_TRUE(adis1657xSpiAccDetect(&acc));
    acc.initFn(&acc);
    EXPECT_EQ(ADIS1657X_ACC_1G_16577, acc.acc_1G);

    fake.reg[ADIS1657X_REG_PROD_ID] = ADIS1657X_PROD_ID_16576;
    adis1657xSpiDetect(&dev);
    acc.initFn(&acc);
    EXPECT_EQ(ADIS1657X_ACC_1G_16576, acc.acc_1G);

    fake.reg[ADIS1657X_REG_PROD_ID] = ADIS1657X_PROD_ID_16575;
    adis1657xSpiDetect(&dev);
    acc.initFn(&acc);
    EXPECT_EQ(ADIS1657X_ACC_1G_16575, acc.acc_1G);
}

// --- read path ---------------------------------------------------------------

TEST_F(Adis1657xTest, FirstReadNegotiatesPolledMode)
{
    initGyro();
    gyro.detectedEXTI = 0;
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
    EXPECT_EQ(0, fake.burstCount);
}

TEST_F(Adis1657xTest, FirstReadPromotesToInterruptModeWithEnoughEdges)
{
    initGyro();
    gyro.detectedEXTI = 5000;
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}

TEST_F(Adis1657xTest, PolledReadUnpacksTheHighWordOfEachGyroAxis)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(1000, -2000, 3000, 0, 0, 0, 0, nextCntr());

    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(1000, gyro.gyroADCRaw[X]);
    EXPECT_EQ(-2000, gyro.gyroADCRaw[Y]);
    EXPECT_EQ(3000, gyro.gyroADCRaw[Z]);
    EXPECT_EQ(1, fake.burstCount);
}

TEST_F(Adis1657xTest, PolledReadConvertsTemperature)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(1, 1, 1, 1, 1, 1, 250, nextCntr());

    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(25, gyro.temperature);
}

TEST_F(Adis1657xTest, DmaReadTakesTheFrameTheHandlerAlreadyFetched)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    fakeSetBurst(-7, 8, -9, 0, 0, 0, 0, nextCntr());
    deliverBurst();

    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(-7, gyro.gyroADCRaw[X]);
    EXPECT_EQ(8, gyro.gyroADCRaw[Y]);
    EXPECT_EQ(-9, gyro.gyroADCRaw[Z]);
    // No transaction of its own: the EXTI handler started it.
    EXPECT_EQ(0, fake.burstCount);
}

TEST_F(Adis1657xTest, ReadRejectsACorruptChecksum)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(1000, 1000, 1000, 0, 0, 0, 0, nextCntr());
    fake.burstFrame[ADIS1657X_BURST_WORD_CHECKSUM] ^= 0x0100;

    EXPECT_FALSE(gyro.readFn(&gyro));
}

TEST_F(Adis1657xTest, ReadRejectsAnAllZeroFrame)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    // Zero is a fixed point of a truncated sum, so the checksum alone accepts
    // this - and a bus with MISO held low is exactly what produces it.
    memset(fake.burstFrame, 0, sizeof(fake.burstFrame));

    EXPECT_FALSE(gyro.readFn(&gyro));
}

TEST_F(Adis1657xTest, ReadRejectsARepeatedSampleCounter)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    const uint16_t c = nextCntr();

    fakeSetBurst(500, 500, 500, 0, 0, 0, 0, c);
    EXPECT_TRUE(gyro.readFn(&gyro));

    // Same counter means the part has not produced a new sample.
    fakeSetBurst(600, 600, 600, 0, 0, 0, 0, c);
    EXPECT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(500, gyro.gyroADCRaw[X]);

    fakeSetBurst(700, 700, 700, 0, 0, 0, 0, nextCntr());
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(700, gyro.gyroADCRaw[X]);
}

TEST_F(Adis1657xTest, ReadAcceptsAFrameWithDiagnosticBitsSet)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(1234, 0, 0, 0, 0, 0, 0, nextCntr());
    // DIAG_STAT is outside the checksum span, and its bits are sticky. Betaflight
    // cannot report or recover from a sensor fault, so dropping flagged frames
    // would turn one transient into a permanent freeze.
    fake.burstFrame[ADIS1657X_BURST_WORD_DIAG_STAT] = __builtin_bswap16(0x0040);

    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(1234, gyro.gyroADCRaw[X]);
}

TEST_F(Adis1657xTest, AccReadTakesTheSameFrameAsTheGyro)
{
    initGyro();
    ASSERT_TRUE(adis1657xSpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, -100, 200, -300, 0, nextCntr());

    ASSERT_TRUE(gyro.readFn(&gyro));
    EXPECT_TRUE(acc.readFn(&acc));
    EXPECT_EQ(-100, acc.ADCRaw[X]);
    EXPECT_EQ(200, acc.ADCRaw[Y]);
    EXPECT_EQ(-300, acc.ADCRaw[Z]);
    // Still one transaction: the accel has no bus of its own.
    EXPECT_EQ(1, fake.burstCount);
}

TEST_F(Adis1657xTest, AccReadIgnoresTheSampleCounter)
{
    initGyro();
    ASSERT_TRUE(adis1657xSpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    fakeSetBurst(0, 0, 0, 11, 22, 33, 0, nextCntr());
    deliverBurst();

    // The gyro owns the counter; a repeat is harmless at the rate the accel is
    // consumed, so the accel must keep accepting the frame.
    EXPECT_TRUE(acc.readFn(&acc));
    EXPECT_TRUE(acc.readFn(&acc));
    EXPECT_EQ(11, acc.ADCRaw[X]);
}

TEST_F(Adis1657xTest, AccReadIsQuietBeforeTheModeIsNegotiated)
{
    initGyro();
    ASSERT_TRUE(adis1657xSpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_INIT;
    EXPECT_TRUE(acc.readFn(&acc));
}

TEST_F(Adis1657xTest, AccReadRejectsACorruptChecksum)
{
    initGyro();
    ASSERT_TRUE(adis1657xSpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    fakeSetBurst(0, 0, 0, 1, 2, 3, 0, nextCntr());
    fake.burstFrame[ADIS1657X_BURST_WORD_CHECKSUM] ^= 0x0001;
    deliverBurst();

    EXPECT_FALSE(acc.readFn(&acc));
}

// --- frame layout ------------------------------------------------------------

TEST_F(Adis1657xTest, BurstFrameLayout)
{
    EXPECT_EQ(36u, (unsigned)ADIS1657X_BURST_FRAME_LEN);
    EXPECT_EQ(0x68, ADIS1657X_BURST_CMD);
    // A low/high pair per axis, gyro before accel.
    EXPECT_EQ(3, ADIS1657X_BURST_GYRO_HIGH(0));
    EXPECT_EQ(7, ADIS1657X_BURST_GYRO_HIGH(2));
    EXPECT_EQ(9, ADIS1657X_BURST_ACCEL_HIGH(0));
    EXPECT_EQ(13, ADIS1657X_BURST_ACCEL_HIGH(2));
    // The checksum covers everything from the first gyro word to the timestamp,
    // excluding FIFO_CNT and DIAG_STAT.
    EXPECT_EQ(2, ADIS1657X_BURST_CHECKSUM_FIRST_WORD);
    EXPECT_EQ(17, ADIS1657X_BURST_CHECKSUM_END_WORD);
}

TEST_F(Adis1657xTest, BurstFrameFitsTheGyroBuffer)
{
    // gyro_init.c splits GYRO_BUF_SIZE in half, one direction each.
    EXPECT_LE((unsigned)ADIS1657X_BURST_FRAME_LEN, 128u / 2u);
}

// --- STUBS -------------------------------------------------------------------

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
void spiWait(const extDevice_t *) {}
void mpuGyroInit(gyroDev_t *) {}
busStatus_e mpuIntCallback(uintptr_t) { return BUS_READY; }

uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}

bool spiUseDMA(const extDevice_t *)
{
    return false;
}

/*
 * Every register frame is two bytes. Bit 7 of the address byte set means write,
 * and a write moves one half of the register at a time. A read frame only latches
 * the answer; the following frame, which carries no address, clocks it out.
 */
void spiReadWriteBuf(const extDevice_t *, uint8_t *txData, uint8_t *rxData, int len)
{
    if (len != 2) {
        return;
    }

    if (!txData && rxData) {
        rxData[0] = fake.pending >> 8;
        rxData[1] = fake.pending & 0xff;
        return;
    }

    if (!txData) {
        return;
    }

    const uint8_t addr = txData[0];

    if (addr & ADIS1657X_WRITE_BIT) {
        const uint8_t reg = addr & 0x7f;
        if ((reg & 1) == 0) {
            fake.pendingLowReg = reg;
            fake.pendingLowByte = txData[1];
            fake.pendingLowValid = true;
        } else if (fake.pendingLowValid && (reg == fake.pendingLowReg + 1)) {
            const uint16_t value = (txData[1] << 8) | fake.pendingLowByte;
            fake.writes.push_back({fake.pendingLowReg, value});
            // GLOB_CMD is a command, not storage: a software reset restores
            // defaults, which the fake models as leaving the map alone.
            if (fake.pendingLowReg != ADIS1657X_REG_GLOB_CMD) {
                fake.reg[fake.pendingLowReg] = value;
            }
            fake.pendingLowValid = false;
        }
        return;
    }

    fake.pending = fake.reg[addr & 0x7f];
}

void spiSequence(const extDevice_t *dev, busSegment_t *segments)
{
    fake.burstCount++;
    if (segments[0].u.buffers.rxData) {
        memcpy(segments[0].u.buffers.rxData, fake.burstFrame, sizeof(fake.burstFrame));
    }
    (void)dev;
}

} // extern "C"
