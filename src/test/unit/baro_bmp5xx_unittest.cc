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
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with Betaflight.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {

#include "platform.h"
#include "target.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_bmp5xx.h"
#include "drivers/bus.h"
#include "drivers/io.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

namespace {

uint8_t stubChipId = 0;
int ioHiCalls = 0;
int ioInitCalls = 0;
int ioConfigCalls = 0;
int spiSetClkDivisorCalls = 0;
int ioPreinitCalls = 0;

void resetStubs(void)
{
    stubChipId = 0;
    ioHiCalls = 0;
    ioInitCalls = 0;
    ioConfigCalls = 0;
    spiSetClkDivisorCalls = 0;
    ioPreinitCalls = 0;
}

baroDev_t makeSpiBaro(void)
{
    static busDevice_t bus;
    baroDev_t baro = {};

    bus = {};
    bus.busType = BUS_TYPE_SPI;

    baro.dev.bus = &bus;
    baro.dev.busType_u.spi.csnPin = (IO_t)1;
    return baro;
}

} // namespace

TEST(baroBmp5xxTest, DetectInitializesSpiBusWhenSupportedSpiDefineIsEnabled)
{
    resetStubs();

    bmp5xxConfig_t config = {};
    baroDev_t baro = makeSpiBaro();
    uint8_t detectedChip = 0;

    stubChipId = 0x50;

    EXPECT_TRUE(bmp5xxDetect(&config, &baro, &detectedChip));
    EXPECT_EQ(0x50, detectedChip);
    EXPECT_EQ(1, ioHiCalls);
    EXPECT_EQ(1, ioInitCalls);
    EXPECT_EQ(1, ioConfigCalls);
    EXPECT_EQ(1, spiSetClkDivisorCalls);
    EXPECT_EQ(0, ioPreinitCalls);
}

TEST(baroBmp5xxTest, DetectDeinitializesSpiBusOnUnknownChip)
{
    resetStubs();

    bmp5xxConfig_t config = {};
    baroDev_t baro = makeSpiBaro();

    EXPECT_FALSE(bmp5xxDetect(&config, &baro, NULL));
    EXPECT_EQ(1, ioHiCalls);
    EXPECT_EQ(1, ioInitCalls);
    EXPECT_EQ(1, ioConfigCalls);
    EXPECT_EQ(1, spiSetClkDivisorCalls);
    EXPECT_EQ(1, ioPreinitCalls);
}

extern "C" {

uint32_t millis(void)
{
    return 0;
}

void delay(uint32_t)
{
}

bool busBusy(const extDevice_t *, bool *)
{
    return false;
}

bool busReadRegisterBuffer(const extDevice_t *, uint8_t reg, uint8_t *data, uint8_t length)
{
    if (reg == 0x01 && length >= 2) {
        data[0] = 0;
        data[1] = stubChipId;
    }
    return true;
}

bool busReadRegisterBufferStart(const extDevice_t *, uint8_t, uint8_t *, uint8_t)
{
    return true;
}

bool busWriteRegister(const extDevice_t *, uint8_t, uint8_t)
{
    return true;
}

bool busWriteRegisterStart(const extDevice_t *, uint8_t, uint8_t)
{
    return true;
}

void busDeviceRegister(const extDevice_t *)
{
}

uint16_t spiCalculateDivider(uint32_t)
{
    return 2;
}

void spiSetClkDivisor(const extDevice_t *, uint16_t)
{
    spiSetClkDivisorCalls++;
}

void ioPreinitByIO(const IO_t, uint8_t, ioPreinitPinState_e)
{
    ioPreinitCalls++;
}

void IOConfigGPIO(IO_t, ioConfig_t)
{
    ioConfigCalls++;
}

void IOHi(IO_t)
{
    ioHiCalls++;
}

IO_t IOGetByTag(ioTag_t)
{
    return IO_NONE;
}

void IOInit(IO_t, resourceOwner_e, uint8_t)
{
    ioInitCalls++;
}

void EXTIHandlerInit(extiCallbackRec_t *, extiHandlerCallback *)
{
}

void EXTIConfig(IO_t, extiCallbackRec_t *, int, ioConfig_t, extiTrigger_t)
{
}

void EXTIEnable(IO_t)
{
}

void EXTIDisable(IO_t)
{
}

}
