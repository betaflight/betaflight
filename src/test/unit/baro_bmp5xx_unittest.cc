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
#include "drivers/bus.h"
#include "drivers/io.h"

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(baroBmp5xxTest, CompilesWithSupportedSpiDefines)
{
    EXPECT_TRUE(true);
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

bool busReadRegisterBuffer(const extDevice_t *, uint8_t, uint8_t *, uint8_t)
{
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
}

void ioPreinitByIO(const IO_t, uint8_t, ioPreinitPinState_e)
{
}

void IOConfigGPIO(IO_t, ioConfig_t)
{
}

void IOHi(IO_t)
{
}

IO_t IOGetByTag(ioTag_t)
{
    return IO_NONE;
}

void IOInit(IO_t, resourceOwner_e, uint8_t)
{
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
