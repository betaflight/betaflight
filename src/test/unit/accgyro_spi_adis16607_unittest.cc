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
#include "drivers/accgyro/accgyro_spi_adis16607.h"
#include "drivers/bus.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

} // extern "C"

#include "unittest_macros.h"
#include "gtest/gtest.h"

// A fake ADIS16607 on the far end of the bus. The SPI stubs at the bottom of the
// file drive it, so the driver's real init and read paths run unmodified.
struct FakeAdis {
    uint16_t reg[0x50];
    std::vector<std::pair<uint8_t, uint16_t>> writes;
    uint16_t burstFrame[ADIS16607_BURST_WORD_COUNT]; // wire order, big-endian
    int burstCount;
    uint16_t dataCntr;
    int dropWrites[0x50]; // writes the part silently swallows, per register

    void reset()
    {
        memset(reg, 0, sizeof(reg));
        reg[ADIS16607_REG_DEV_ID] = ADIS16607_DEV_ID_CONST;
        writes.clear();
        memset(burstFrame, 0, sizeof(burstFrame));
        burstCount = 0;
        dataCntr = 0;
        memset(dropWrites, 0, sizeof(dropWrites));
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

    int indexOfLastWrite(uint8_t r) const
    {
        int found = -1;
        for (size_t i = 0; i < writes.size(); i++) {
            if (writes[i].first == r) {
                found = (int)i;
            }
        }
        return found;
    }
};

static FakeAdis fake;

// Build a burst frame in the device's own wire format: big-endian words with a
// trailing truncated sum over DIAG_STAT..DATA_CNTR. The data counter advances on
// every frame, as it does on the part whenever the output registers are rewritten.
static void fakeSetBurst(int16_t ax, int16_t ay, int16_t az,
                         int16_t gx, int16_t gy, int16_t gz, int16_t temp)
{
    uint16_t host[ADIS16607_BURST_WORD_COUNT] = {};
    host[ADIS16607_BURST_WORD_ACCEL + 0] = (uint16_t)ax;
    host[ADIS16607_BURST_WORD_ACCEL + 1] = (uint16_t)ay;
    host[ADIS16607_BURST_WORD_ACCEL + 2] = (uint16_t)az;
    host[ADIS16607_BURST_WORD_GYRO + 0] = (uint16_t)gx;
    host[ADIS16607_BURST_WORD_GYRO + 1] = (uint16_t)gy;
    host[ADIS16607_BURST_WORD_GYRO + 2] = (uint16_t)gz;
    host[ADIS16607_BURST_WORD_TEMPERATURE] = (uint16_t)temp;
    host[ADIS16607_BURST_WORD_DATA_CNTR] = ++fake.dataCntr;

    uint16_t sum = 0;
    for (int i = ADIS16607_BURST_CHECKSUM_FIRST_WORD; i < ADIS16607_BURST_WORD_CHECKSUM; i++) {
        sum += host[i];
    }
    host[ADIS16607_BURST_WORD_CHECKSUM] = sum;

    for (int i = 0; i < ADIS16607_BURST_WORD_COUNT; i++) {
        fake.burstFrame[i] = __builtin_bswap16(host[i]);
    }
}

class AdisTest : public ::testing::Test {
protected:
    gyroDev_t gyro;
    accDev_t acc;
    uint8_t txBuf[ADIS16607_BURST_FRAME_LEN];
    uint8_t rxBuf[ADIS16607_BURST_FRAME_LEN];

    void SetUp() override
    {
        fake.reset();
        memset(&gyro, 0, sizeof(gyro));
        memset(&acc, 0, sizeof(acc));
        memset(txBuf, 0, sizeof(txBuf));
        memset(rxBuf, 0, sizeof(rxBuf));
        gyro.dev.txBuf = txBuf;
        gyro.dev.rxBuf = rxBuf;
        gyro.mpuDetectionResult.sensor = ADIS16607_SPI;
        acc.mpuDetectionResult.sensor = ADIS16607_SPI;
        acc.gyro = &gyro;
    }

    void initGyro()
    {
        ASSERT_TRUE(adis16607SpiGyroDetect(&gyro));
        gyro.initFn(&gyro);
    }
};

// --- detection ---------------------------------------------------------------

TEST_F(AdisTest, DetectMatchesDevId)
{
    extDevice_t dev = {};
    EXPECT_EQ(ADIS16607_SPI, adis16607SpiDetect(&dev));
}

TEST_F(AdisTest, DetectRejectsWrongDevId)
{
    extDevice_t dev = {};
    fake.reg[ADIS16607_REG_DEV_ID] = 0x1234;
    EXPECT_EQ(MPU_NONE, adis16607SpiDetect(&dev));
}

TEST_F(AdisTest, DetectEntersHalfDuplexBeforeReading)
{
    // Out of reset a bare 32-bit read is parsed as full duplex and answers with a
    // CRC error, so the probe must write the half-duplex key first.
    extDevice_t dev = {};
    adis16607SpiDetect(&dev);
    ASSERT_FALSE(fake.writes.empty());
    EXPECT_EQ(ADIS16607_REG_SPI_HALFDUPLEX_KEY, fake.writes[0].first);
    EXPECT_EQ(ADIS16607_SPI_HALFDUPLEX_KEY_VAL, fake.writes[0].second);
}

TEST_F(AdisTest, GyroDetectRejectsOtherSensor)
{
    gyro.mpuDetectionResult.sensor = MPU_NONE;
    EXPECT_FALSE(adis16607SpiGyroDetect(&gyro));
}

TEST_F(AdisTest, AccDetectRejectsOtherSensor)
{
    acc.mpuDetectionResult.sensor = MPU_NONE;
    EXPECT_FALSE(adis16607SpiAccDetect(&acc));
}

// --- init --------------------------------------------------------------------

TEST_F(AdisTest, InitPublishesScales)
{
    initGyro();
    EXPECT_FLOAT_EQ(1.0f / ADIS16607_GYRO_LSB_PER_DPS, gyro.scale);
    EXPECT_FLOAT_EQ(ADIS16607_TEMP_SCALE, gyro.tempScale);
    EXPECT_FLOAT_EQ(ADIS16607_TEMP_ZERO, gyro.tempZero);
}

TEST_F(AdisTest, VariantSelectsTheGyroSensitivity)
{
    // Only the -2 and -3 exist, and no register distinguishes them, so the scale
    // rides on ADIS16607_VARIANT alone. Getting it wrong is a silent 4x on all
    // three axes, so pin the selected sensitivity and check the built scale is
    // its reciprocal.
    initGyro();
#if ADIS16607_VARIANT == 2
    EXPECT_FLOAT_EQ(62.5f, ADIS16607_GYRO_LSB_PER_DPS);
    EXPECT_FLOAT_EQ(1.0f / 62.5f, gyro.scale);
#elif ADIS16607_VARIANT == 3
    EXPECT_FLOAT_EQ(15.625f, ADIS16607_GYRO_LSB_PER_DPS);
    EXPECT_FLOAT_EQ(1.0f / 15.625f, gyro.scale);
#else
#error "test needs updating for a new ADIS16607_VARIANT"
#endif

    // A reading at the part's maximum must not clip int16_t gyroADCRaw[] on
    // either variant: 480 deg/s * 62.5 = 30000, 2000 deg/s * 15.625 = 31250.
    EXPECT_LT(480.0f * 62.5f, 32767.0f);
    EXPECT_LT(2000.0f * 15.625f, 32767.0f);
}

TEST_F(AdisTest, InitConfiguresDataAndRate)
{
    initGyro();
    EXPECT_TRUE(fake.wrote(ADIS16607_REG_USER_DATA_CFG, ADIS16607_USER_DATA_CFG));
    EXPECT_TRUE(fake.wrote(ADIS16607_REG_DEC_RATE, ADIS16607_DEC_RATE_4780HZ));
    EXPECT_TRUE(fake.wrote(ADIS16607_REG_USER_FIFO_CFG, ADIS16607_FIFO_CFG_DISABLED));
    EXPECT_TRUE(fake.wrote(ADIS16607_REG_USER_GPIO_CFG1, ADIS16607_GPIO_CFG1_GPIO3_DATA_RDY));
}

TEST_F(AdisTest, InitZeroesDecRateBeforeSettingIt)
{
    // The decimation accumulator only resets through a zero write.
    initGyro();
    int zeroAt = -1, setAt = -1;
    for (size_t i = 0; i < fake.writes.size(); i++) {
        if (fake.writes[i].first != ADIS16607_REG_DEC_RATE) {
            continue;
        }
        if (fake.writes[i].second == ADIS16607_DEC_RATE_9560HZ && zeroAt < 0) {
            zeroAt = (int)i;
        }
        if (fake.writes[i].second == ADIS16607_DEC_RATE_4780HZ) {
            setAt = (int)i;
        }
    }
    ASSERT_GE(zeroAt, 0);
    ASSERT_GE(setAt, 0);
    EXPECT_LT(zeroAt, setAt);
}

TEST_F(AdisTest, InitLocksLast)
{
    // The part will not leave its initialisation phase until write lock is set,
    // so the lock has to come after every configuration write.
    initGyro();
    const int lockAt = fake.indexOfLastWrite(ADIS16607_REG_WRITE_LOCK);
    ASSERT_GE(lockAt, 0);
    EXPECT_EQ((size_t)lockAt, fake.writes.size() - 1);
    EXPECT_EQ(ADIS16607_WRITE_LOCK_KEY_A, fake.writes[lockAt].second);
    EXPECT_EQ(ADIS16607_WRITE_LOCK_KEY_B, fake.writes[lockAt - 1].second);
}

TEST_F(AdisTest, InitOverridesMpuDefaultRegisters)
{
    initGyro();
    EXPECT_EQ(ADIS16607_REG_X_GYRO_HIGH, gyro.gyroDataReg);
    EXPECT_EQ(ADIS16607_REG_X_ACCEL_HIGH, gyro.accDataReg);
    EXPECT_EQ(ADIS16607_REG_TEMPERATURE, gyro.tempDataReg);
    EXPECT_EQ(ADIS16607_BURST_CMD, gyro.dmaReadRegStart);
}

TEST_F(AdisTest, AccInitPublishesOneG)
{
    ASSERT_TRUE(adis16607SpiAccDetect(&acc));
    acc.initFn(&acc);
    EXPECT_EQ(ADIS16607_ACC_1G, acc.acc_1G);
}

// --- readFn mode negotiation -------------------------------------------------

TEST_F(AdisTest, ReadInitFallsBackToPolledWithoutExti)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INIT;
    gyro.detectedEXTI = 0;
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
}

TEST_F(AdisTest, ReadInitPromotesToInterruptWithExti)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INIT;
    gyro.detectedEXTI = 5000;
    EXPECT_TRUE(gyro.readFn(&gyro));
    // The USE_DMA branch is compiled out here: enabling it needs DMA types the
    // unit-test platform shim does not provide, so the DMA segment setup is
    // covered only on hardware.
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}

TEST_F(AdisTest, ReadInitZeroFillsTxBuf)
{
    // Trailing 0xff bytes would look like further read commands to the part.
    initGyro();
    memset(txBuf, 0xff, sizeof(txBuf));
    gyro.gyroModeSPI = GYRO_EXTI_INIT;
    gyro.readFn(&gyro);
    for (size_t i = 1; i < sizeof(txBuf); i++) {
        EXPECT_EQ(0, txBuf[i]) << "txBuf[" << i << "]";
    }
}

// --- burst unpack ------------------------------------------------------------

TEST_F(AdisTest, ReadUnpacksGyroAxesAndTemperature)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, 1000, -2000, 3000, 1000);

    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(1000, gyro.gyroADCRaw[X]);
    EXPECT_EQ(-2000, gyro.gyroADCRaw[Y]);
    EXPECT_EQ(3000, gyro.gyroADCRaw[Z]);
    // 1000 LSB * 0.005 degC/LSB + 25 degC
    EXPECT_EQ(30, gyro.temperature);
}

TEST_F(AdisTest, ReadIssuesBurstCommand)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, 0, 0, 0, 0);
    gyro.readFn(&gyro);
    EXPECT_EQ(ADIS16607_BURST_CMD, txBuf[0]);
    EXPECT_EQ(1, fake.burstCount);
}

TEST_F(AdisTest, ReadRejectsBadChecksum)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, 111, 222, 333, 0);
    fake.burstFrame[ADIS16607_BURST_WORD_CHECKSUM] ^= 0x0100;

    EXPECT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(0, gyro.gyroADCRaw[X]); // rejected frame must not be published
}

TEST_F(AdisTest, ReadNegativeFullScaleSurvivesSignExtension)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, -32768, 32767, -1, 0);
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(-32768, gyro.gyroADCRaw[X]);
    EXPECT_EQ(32767, gyro.gyroADCRaw[Y]);
    EXPECT_EQ(-1, gyro.gyroADCRaw[Z]);
}

TEST_F(AdisTest, DmaModeDoesNotStartATransfer)
{
    // In DMA mode the EXTI handler owns the transfer; readFn only unpacks.
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    fakeSetBurst(0, 0, 0, 7, 8, 9, 0);
    memcpy(rxBuf, fake.burstFrame, sizeof(fake.burstFrame)); // as the EXTI-driven DMA would have
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(0, fake.burstCount);
    EXPECT_EQ(7, gyro.gyroADCRaw[X]);
}

TEST_F(AdisTest, ReadRejectsRepeatedDataCounter)
{
    // A re-read of output registers the part has not rewritten carries a valid
    // checksum and the previous counter. It is a duplicate, not a sample.
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, 1234, 0, 0, 0);
    ASSERT_TRUE(gyro.readFn(&gyro));

    EXPECT_FALSE(gyro.readFn(&gyro));
    EXPECT_EQ(1234, gyro.gyroADCRaw[X]); // still the accepted sample
}

TEST_F(AdisTest, ReadAcceptsAdvancedDataCounter)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(0, 0, 0, 1234, 0, 0, 0);
    ASSERT_TRUE(gyro.readFn(&gyro));

    fakeSetBurst(0, 0, 0, 4321, 0, 0, 0);
    EXPECT_TRUE(gyro.readFn(&gyro));
    EXPECT_EQ(4321, gyro.gyroADCRaw[X]);
}

TEST_F(AdisTest, ReadRejectsStuckHighFrame)
{
    // 0xffff on every word sums to a matching truncated checksum, so MISO stuck
    // high has to be rejected on its content, as MISO stuck low is.
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    memset(rxBuf, 0xff, sizeof(rxBuf));
    EXPECT_FALSE(gyro.readFn(&gyro));
}

TEST_F(AdisTest, ReadRejectsStuckLowFrame)
{
    initGyro();
    gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
    memset(rxBuf, 0, sizeof(rxBuf));
    EXPECT_FALSE(gyro.readFn(&gyro));
}

TEST_F(AdisTest, InitRetriesDroppedConfigWrites)
{
    // The part swallows writes while locked or while its bootloader runs. A
    // dropped USER_DATA_CFG would shift every word of the burst, so init has to
    // read each configuration register back and write it again until it takes.
    fake.dropWrites[ADIS16607_REG_USER_DATA_CFG] = 3;
    fake.dropWrites[ADIS16607_REG_DEC_RATE] = 3;
    initGyro();
    EXPECT_EQ(ADIS16607_USER_DATA_CFG, fake.reg[ADIS16607_REG_USER_DATA_CFG]);
    EXPECT_EQ(ADIS16607_DEC_RATE_4780HZ, fake.reg[ADIS16607_REG_DEC_RATE]);
}

// --- accel shares the gyro's frame -------------------------------------------

TEST_F(AdisTest, AccReadsGyroFrameWithoutOwnTransfer)
{
    initGyro();
    ASSERT_TRUE(adis16607SpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(11, -22, 33, 0, 0, 0, 0);
    ASSERT_TRUE(gyro.readFn(&gyro));
    const int burstsAfterGyroRead = fake.burstCount;

    EXPECT_TRUE(acc.readFn(&acc));
    EXPECT_EQ(11, acc.ADCRaw[X]);
    EXPECT_EQ(-22, acc.ADCRaw[Y]);
    EXPECT_EQ(33, acc.ADCRaw[Z]);
    EXPECT_EQ(burstsAfterGyroRead, fake.burstCount);
}

TEST_F(AdisTest, AccReadSucceedsBeforeFirstBurst)
{
    // acc readFn can be called while readFn is still negotiating its mode; the
    // buffer holds no frame yet, so it must not fail the sensor.
    ASSERT_TRUE(adis16607SpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_INIT;
    EXPECT_TRUE(acc.readFn(&acc));
}

TEST_F(AdisTest, AccReadRejectsBadChecksum)
{
    initGyro();
    ASSERT_TRUE(adis16607SpiAccDetect(&acc));
    gyro.gyroModeSPI = GYRO_EXTI_NO_INT;
    fakeSetBurst(1, 2, 3, 0, 0, 0, 0);
    ASSERT_TRUE(gyro.readFn(&gyro));
    fake.burstFrame[ADIS16607_BURST_WORD_CHECKSUM] ^= 0x0100;
    memcpy(rxBuf, fake.burstFrame, sizeof(fake.burstFrame));
    EXPECT_FALSE(acc.readFn(&acc));
}

// --- frame layout ------------------------------------------------------------

TEST_F(AdisTest, BurstFrameLayout)
{
    EXPECT_EQ(22u, (unsigned)ADIS16607_BURST_FRAME_LEN);
    EXPECT_EQ(0x85, ADIS16607_BURST_CMD);
    EXPECT_EQ(ADIS16607_BURST_WORD_ACCEL + 3, ADIS16607_BURST_WORD_GYRO);
    // The payload starts at byte 0 of the frame, with no leading dummy byte to
    // skip - unlike the MPU-style drivers, which read from &rxBuf[1].
    EXPECT_EQ(0, ADIS16607_BURST_WORD_STATUS);
}

TEST_F(AdisTest, UserDataCfgSelectsSixAxesPlusTemperatureIn16Bit)
{
    const uint16_t cfg = ADIS16607_USER_DATA_CFG;
    EXPECT_EQ(0, cfg & ADIS16607_DATA_CFG_WORD_SIZE_32);
    EXPECT_TRUE(cfg & ADIS16607_DATA_CFG_Z_GYRO_EN);
    EXPECT_TRUE(cfg & ADIS16607_DATA_CFG_TEMPERATURE_EN);
    // Extra words would cost bus time in the EXTI-to-sample path.
    EXPECT_FALSE(cfg & ADIS16607_DATA_CFG_X_DELTVEL_EN);
    EXPECT_FALSE(cfg & ADIS16607_DATA_CFG_X_DELTANG_EN);
    EXPECT_FALSE(cfg & ADIS16607_DATA_CFG_TIME_STAMP_EN);
    // The counter is the exception: it is what makes a duplicated frame visible.
    EXPECT_TRUE(cfg & ADIS16607_DATA_CFG_DATA_CNTR_EN);
}

// --- STUBS -------------------------------------------------------------------

extern "C" {

void delay(uint32_t) {}
void spiSetClkDivisor(const extDevice_t *, uint16_t) {}
void spiWait(const extDevice_t *) {}
void spiDmaEnable(const extDevice_t *, bool) {}
void mpuGyroInit(gyroDev_t *) {}
void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t) {}
void EXTIEnable(IO_t) {}
busStatus_e mpuIntCallback(uintptr_t) { return BUS_READY; }

uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}

bool spiUseDMA(const extDevice_t *)
{
    return false;
}

IO_t IOGetByTag(ioTag_t)
{
    return NULL;
}

// Half-duplex register access: a 3-byte frame is a write, a 4-byte frame is a
// read whose response lands in the same frame.
void spiReadWriteBuf(const extDevice_t *, uint8_t *txData, uint8_t *rxData, int len)
{
    if (len == 3) {
        const uint8_t reg = txData[0] & 0x7f;
        const uint16_t value = (txData[1] << 8) | txData[2];
        fake.writes.push_back({reg, value});
        if (reg == ADIS16607_REG_SOFT_RESET) {
            return; // reset restores defaults, which the fake models as "no change"
        }
        if (fake.dropWrites[reg] > 0) {
            fake.dropWrites[reg]--; // as the part does while locked or booting
            return;
        }
        fake.reg[reg] = value;
    } else if (len == 4 && rxData) {
        const uint8_t reg = txData[0] & 0x7f;
        const uint16_t value = fake.reg[reg];
        rxData[0] = 0;
        rxData[1] = 0;
        rxData[2] = value >> 8;
        rxData[3] = value & 0xff;
    }
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
