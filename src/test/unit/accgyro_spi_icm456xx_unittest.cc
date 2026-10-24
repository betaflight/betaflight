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
#include <stddef.h>
#include <map>
#include <vector>

extern "C" {

#include "platform.h"
#include "target.h"
#include "build/build_config.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_icm456xx.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "sensors/gyro.h"

PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);

uint8_t armingFlags = 0;

} // extern "C"

#include "unittest_macros.h"
#include "gtest/gtest.h"

// The registers the indirect accesses go through, from the driver. A burst
// starting at IREG_ADDR_15_8 covers IREG_ADDR_7_0 at 0x7D and IREG_DATA next.
static const uint8_t REG_IREG_ADDR_15_8 = 0x7C;
static const uint8_t REG_IREG_DATA = 0x7E;
static const uint8_t REG_MISC2 = 0x7F;

// The gyro data read: the register with the SPI read bit, then six data bytes.
static const uint8_t REG_GYRO_DATA_X1_UI = 0x06;
static const uint8_t SPI_READ_BIT = 0x80;
static const uint8_t SPI_BUFFER_SIZE = 7;

// The interrupt count the driver needs before it trusts the interrupt line.
static const uint32_t EXTI_DETECT_THRESHOLD = 1000;

// The starting read offset every DMA gyro driver uses.
static const int32_t GYRO_DMA_START_DURATION = 5;

// The two source control registers, and the fields inside them. ACCEL_SRC_CTRL
// is bits 1:0 of IPREG_SYS2_REG_123 and GYRO_SRC_CTRL is bits 6:5 of
// IPREG_SYS1_REG_166, per the vendor register map. Value 2 is "interpolator on,
// anti-alias FIR on", value 3 is reserved.
static const uint16_t ACCEL_SRC_CTRL_ADDR = 0xA57B;
static const uint16_t GYRO_SRC_CTRL_ADDR = 0xA4A6;
static const uint8_t SRC_CTRL_INTERP_ON_FIR_ON = 0x02;

// Reset values with bits set outside the fields the driver writes, so a whole
// byte write shows up as those bits being lost.
static const uint8_t ACCEL_SRC_CTRL_RESET = 0x14;
static const uint8_t GYRO_SRC_CTRL_RESET = 0x18;

// A fake part: an indirect register space, plus the traffic that reached it.
struct FakeIcm456xx {
    std::map<uint16_t, uint8_t> ireg;
    std::vector<size_t> burstLengths;   // one entry per write burst to 0x7C
    int singleByteWrites;               // writes through spiWriteReg to 0x7C..0x7E
    uint16_t address;                   // the address the last burst selected

    void reset()
    {
        ireg.clear();
        ireg[ACCEL_SRC_CTRL_ADDR] = ACCEL_SRC_CTRL_RESET;
        ireg[GYRO_SRC_CTRL_ADDR] = GYRO_SRC_CTRL_RESET;
        burstLengths.clear();
        singleByteWrites = 0;
        address = 0;
    }
};

static FakeIcm456xx fake;

// What spiUseDMA() answers for the gyro's bus.
static bool busHasDma;

class AccgyroSpiIcm456xx : public ::testing::Test
{
protected:
    void SetUp() override
    {
        fake.reset();
        gyro = {};
        gyro.mpuDetectionResult.sensor = ICM_45686_SPI;
        acc = {};
        acc.gyro = &gyro;
        acc.mpuDetectionResult.sensor = ICM_45686_SPI;
    }

    gyroDev_t gyro;
    accDev_t acc;
};

// The accelerometer's field is at bits 1:0, and the rest of the register has to
// survive. A driver that composes the value out of two independent bits writes
// 3 here, which the datasheet calls reserved, and the part answers with a random
// per-boot offset on one accelerometer axis.
TEST_F(AccgyroSpiIcm456xx, AccelSrcCtrlIsADocumentedValue)
{
    icm456xxAccInit(&acc);

    const uint8_t value = fake.ireg[ACCEL_SRC_CTRL_ADDR];
    EXPECT_EQ(SRC_CTRL_INTERP_ON_FIR_ON, value & 0x03);
    EXPECT_EQ(ACCEL_SRC_CTRL_RESET & ~0x03, value & ~0x03);
}

// The gyro's field is at bits 6:5 of a different register. Writing the value at
// bits 1:0 there leaves the field at 0, which is the filter and the interpolator
// switched off, and disturbs the two bits that carry GYRO_AFSR_MODE.
TEST_F(AccgyroSpiIcm456xx, GyroSrcCtrlIsWrittenAtItsOwnBitPosition)
{
    icm456xxGyroInit(&gyro);

    const uint8_t value = fake.ireg[GYRO_SRC_CTRL_ADDR];
    EXPECT_EQ(SRC_CTRL_INTERP_ON_FIR_ON, (value >> 5) & 0x03);
    EXPECT_EQ(GYRO_SRC_CTRL_RESET & ~(0x03 << 5), value & ~(0x03 << 5));
}

// The part prefetches on every chip select rise, so the two address bytes and
// the data byte have to reach it inside one chip select. Three separate register
// writes are three chip selects and the write lands somewhere else.
TEST_F(AccgyroSpiIcm456xx, IndirectWritesGoOutAsOneBurst)
{
    icm456xxAccInit(&acc);

    EXPECT_EQ(0, fake.singleByteWrites);
    ASSERT_FALSE(fake.burstLengths.empty());
    for (const size_t length : fake.burstLengths) {
        // Two address bytes for a read, two plus the data byte for a write.
        EXPECT_TRUE(length == 2 || length == 3) << "burst of " << length << " bytes";
    }
}

static uint8_t txBuf[16];
static uint8_t rxBuf[16];

static void readInInitMode(gyroDev_t *gyro, uint32_t detectedEXTI, bool hasDma)
{
    gyro->dev.txBuf = txBuf;
    gyro->dev.rxBuf = rxBuf;
    gyro->gyroModeSPI = GYRO_EXTI_INIT;
    gyro->detectedEXTI = detectedEXTI;
    busHasDma = hasDma;
    ASSERT_TRUE(icm456xxSpiGyroDetect(gyro));
    gyro->readFn(gyro);
}

// A board with no interrupt line, or too few interrupts, has to poll the sensor.
TEST_F(AccgyroSpiIcm456xx, TooFewInterruptsFallBackToPolling)
{
    readInInitMode(&gyro, 0, true);
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);

    readInInitMode(&gyro, EXTI_DETECT_THRESHOLD, true);
    EXPECT_EQ(GYRO_EXTI_NO_INT, gyro.gyroModeSPI);
}

TEST_F(AccgyroSpiIcm456xx, InterruptsWithDmaSetUpTheDmaRead)
{
    readInInitMode(&gyro, EXTI_DETECT_THRESHOLD + 1, true);

    EXPECT_EQ(GYRO_EXTI_INT_DMA, gyro.gyroModeSPI);
    EXPECT_EQ(GYRO_DMA_START_DURATION, gyro.gyroDmaMaxDuration);
    EXPECT_EQ(REG_GYRO_DATA_X1_UI | SPI_READ_BIT, txBuf[0]);
    EXPECT_EQ(SPI_BUFFER_SIZE, gyro.segments[0].len);
    EXPECT_EQ(txBuf, gyro.segments[0].u.buffers.txData);
    EXPECT_EQ(rxBuf, gyro.segments[0].u.buffers.rxData);
}

TEST_F(AccgyroSpiIcm456xx, InterruptsWithoutDmaReadOnInterrupt)
{
    readInInitMode(&gyro, EXTI_DETECT_THRESHOLD + 1, false);
    EXPECT_EQ(GYRO_EXTI_INT, gyro.gyroModeSPI);
}

// --- STUBS: the fake part, and the symbols the driver TU pulls in ------------

extern "C" {

void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}

void spiSetClkDivisor(const extDevice_t *, uint16_t) {}

uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}

bool spiUseDMA(const extDevice_t *)
{
    return busHasDma;
}

void spiWriteReg(const extDevice_t *, uint8_t reg, uint8_t)
{
    if (reg >= REG_IREG_ADDR_15_8 && reg <= REG_IREG_DATA) {
        fake.singleByteWrites++;
    }
}

void spiWriteRegBuf(const extDevice_t *, uint8_t reg, uint8_t *data, uint32_t length)
{
    if (reg != REG_IREG_ADDR_15_8) {
        return;
    }
    fake.burstLengths.push_back(length);
    if (length >= 2) {
        fake.address = (data[0] << 8) | data[1];
    }
    if (length >= 3) {
        fake.ireg[fake.address] = data[2];
    }
}

uint8_t spiReadRegMsk(const extDevice_t *, uint8_t reg)
{
    switch (reg) {
    case REG_MISC2:
        return 0x01;  // IREG_DONE, and no soft reset in progress
    case REG_IREG_DATA:
        return fake.ireg[fake.address];
    default:
        return 0;
    }
}

bool spiReadRegMskBufRB(const extDevice_t *, uint8_t, uint8_t *, uint8_t)
{
    return false;
}

void mpuGyroInit(gyroDev_t *) {}

bool mpuGyroReadSPI(gyroDev_t *)
{
    return false;
}

bool mpuAccReadSPI(accDev_t *)
{
    return false;
}

busStatus_e mpuIntCallback(uintptr_t)
{
    return BUS_READY;
}

} // extern "C"
