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

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "drivers/serial.h"
    #include "io/serial.h"

    void serialInit(serialConfig_t *initialSerialConfig);

}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//uint32_t testFeatureMask = 0;
uint8_t cliMode = 0;

TEST(IoSerialTest, TestFindPortConfig)
{
    // given
    serialConfig_t serialConfig;
    memset(&serialConfig, 0, sizeof(serialConfig));

    // when
    serialInit(&serialConfig);

    // and
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);

    // then
    EXPECT_EQ(NULL, portConfig);
}


// STUBS

extern "C" {
//
//bool feature(uint32_t mask) {
//    return (mask & testFeatureMask);
//}s

void delay(uint32_t) {}
void cliEnter(serialPort_t *) {}
void cliProcess(void) {}
bool isSerialTransmitBufferEmpty(serialPort_t *) {
    return true;
}
void mspProcess(void) {}
void systemResetToBootloader(void) {}

}
