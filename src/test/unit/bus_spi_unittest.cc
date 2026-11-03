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

#include <cstring>

extern "C" {
#include "platform.h"
#include "target.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/io.h"

extern busDevice_t spiBusDevice[SPIDEV_COUNT];
}

#include "gtest/gtest.h"

namespace {

int abortDmaCalls;
int sequenceStartCalls;
const extDevice_t *lastStartedDev;
IO_t lastNegatedCs;

const IO_t gyroCs = (IO_t)1;
const IO_t otherCs = (IO_t)2;

class SpiDmaRecoveryTest : public ::testing::Test {
protected:
    gyroDev_t gyro;
    extDevice_t other;
    uint8_t txBuf[16];
    uint8_t rxBuf[16];
    busSegment_t otherSegments[2];

    void SetUp() override
    {
        abortDmaCalls = 0;
        sequenceStartCalls = 0;
        lastStartedDev = nullptr;
        lastNegatedCs = IO_NONE;

        memset(spiBusDevice, 0, sizeof(spiBusDevice));
        memset(spiDevice, 0, sizeof(spiDevice));
        spiBusDevice[0].busType = BUS_TYPE_SPI;
        spiBusDevice[0].curSegment = (busSegment_t *)BUS_SPI_FREE;

        memset(&gyro, 0, sizeof(gyro));
        gyro.dev.bus = &spiBusDevice[0];
        gyro.dev.busType_u.spi.csnPin = gyroCs;
        gyro.dev.callbackArg = (uintptr_t)&gyro;
        gyro.gyroModeSPI = GYRO_EXTI_INT_DMA;
        gyro.segments[0] = {.u.buffers = {txBuf, rxBuf}, 15, true, mpuIntCallback};
        gyro.segments[1] = {.u.link = {nullptr, nullptr}, 0, true, nullptr};

        memset(&other, 0, sizeof(other));
        other.bus = &spiBusDevice[0];
        other.busType_u.spi.csnPin = otherCs;
        otherSegments[0] = {.u.buffers = {txBuf, nullptr}, 256, true, nullptr};
        otherSegments[1] = {.u.link = {nullptr, nullptr}, 0, true, nullptr};
    }

    bool gyroInFlight() const
    {
        return spiBusDevice[0].curSegment == gyro.segments;
    }
};

TEST_F(SpiDmaRecoveryTest, AbortOnIdleBusDoesNothing)
{
    spiAbortTransfer(&gyro.dev);

    EXPECT_EQ(0, abortDmaCalls);
    EXPECT_EQ(IO_NONE, lastNegatedCs);
    EXPECT_EQ(0, spiGetErrorCounter());
}

TEST_F(SpiDmaRecoveryTest, AbortReleasesBusAndCountsError)
{
    spiSequence(&gyro.dev, gyro.segments);
    ASSERT_TRUE(gyroInFlight());

    spiAbortTransfer(&gyro.dev);

    EXPECT_EQ(1, abortDmaCalls);
    EXPECT_EQ(gyroCs, lastNegatedCs);
    EXPECT_FALSE(spiIsBusy(&gyro.dev));
    EXPECT_EQ(1, spiGetErrorCounter());
    EXPECT_FALSE(gyro.dataReady);
}

TEST_F(SpiDmaRecoveryTest, AbortStartsTransferQueuedBehindIt)
{
    spiSequence(&gyro.dev, gyro.segments);
    spiSequence(&other, otherSegments);
    ASSERT_EQ(1, sequenceStartCalls);

    spiAbortTransfer(&gyro.dev);

    EXPECT_EQ(2, sequenceStartCalls);
    EXPECT_EQ(&other, lastStartedDev);
    EXPECT_EQ(otherSegments, spiBusDevice[0].curSegment);
    EXPECT_EQ(nullptr, gyro.segments[1].u.link.dev);
    EXPECT_EQ(nullptr, gyro.segments[1].u.link.segments);
}

TEST_F(SpiDmaRecoveryTest, CompletingGyroReadsAreNeverAbandoned)
{
    for (int i = 0; i < 100; i++) {
        mpuGyroStartDmaRead(&gyro);
        ASSERT_TRUE(gyroInFlight());
        spiIrqHandler(&gyro.dev);
        ASSERT_FALSE(spiIsBusy(&gyro.dev));
    }

    EXPECT_EQ(0, abortDmaCalls);
    EXPECT_EQ(100, sequenceStartCalls);
}

TEST_F(SpiDmaRecoveryTest, StalledGyroReadIsAbandonedAndRestarted)
{
    mpuGyroStartDmaRead(&gyro);
    ASSERT_EQ(1, sequenceStartCalls);

    // The DMA completion never arrives, but data ready interrupts continue
    int interrupts = 0;
    while (abortDmaCalls == 0 && interrupts < 100) {
        mpuGyroStartDmaRead(&gyro);
        interrupts++;
    }

    EXPECT_EQ(1, abortDmaCalls);
    EXPECT_GT(interrupts, 1);
    EXPECT_LE(interrupts, 8);
    EXPECT_EQ(1, spiGetErrorCounter());
    EXPECT_EQ(2, sequenceStartCalls);
    EXPECT_TRUE(gyroInFlight());

    spiIrqHandler(&gyro.dev);
    EXPECT_TRUE(gyro.dataReady);
}

TEST_F(SpiDmaRecoveryTest, GyroQueuedBehindAnotherDeviceIsNotAbandoned)
{
    spiSequence(&other, otherSegments);

    for (int i = 0; i < 100; i++) {
        mpuGyroStartDmaRead(&gyro);
    }

    EXPECT_EQ(0, abortDmaCalls);
    EXPECT_EQ(otherSegments, spiBusDevice[0].curSegment);
    EXPECT_EQ(&gyro.dev, otherSegments[1].u.link.dev);
}

} // namespace

extern "C" {

void spiSequenceStart(const extDevice_t *dev)
{
    sequenceStartCalls++;
    lastStartedDev = dev;
}

void spiInternalAbortDMA(const extDevice_t *) { abortDmaCalls++; }
void spiInitDevice(spiDevice_e) {}
void spiInternalInitStream(const extDevice_t *, volatile busSegment_t *) {}
void spiInternalStartDMA(const extDevice_t *) {}
bool spiInternalReadWriteBufPolled(spiResource_t *, const uint8_t *, uint8_t *, int) { return true; }

void IOHi(IO_t io) { lastNegatedCs = io; }
void IOLo(IO_t) {}
bool IORead(IO_t) { return false; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
IO_t IOGetByTag(ioTag_t) { return IO_NONE; }
void ioPreinitByIO(const IO_t, uint8_t, ioPreinitPinState_e) {}
void ioPreinitByTag(ioTag_t, uint8_t, ioPreinitPinState_e) {}

void EXTIHandlerInit(extiCallbackRec_t *, extiHandlerCallback *) {}
void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t) {}
void EXTIEnable(IO_t) {}

uint32_t getCycleCounter(void) { return 0; }
uint32_t millis(void) { return 1000; }
void delay(uint32_t) {}
void delayMicroseconds(uint32_t) {}

uint16_t spiCalculateDivider(uint32_t) { return 2; }
void busDeviceRegister(const extDevice_t *) {}
bool busReadRegisterBuffer(const extDevice_t *, uint8_t, uint8_t *, uint8_t) { return false; }
bool busWriteRegister(const extDevice_t *, uint8_t, uint8_t) { return false; }

} // extern "C"
