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
    #include <platform.h>

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"

    #include "drivers/serial.h"
    #include "drivers/serial_softserial.h"
    #include "io/serial.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

//uint32_t testFeatureMask = 0;
uint8_t cliMode = 0;

PG_REGISTER(serialConfig_t, serialConfig, PG_SERIAL_CONFIG, 0);

extern uint8_t serialPortCount;

TEST(IoSerialTest, TestSoftSerialPortsEnabled)
{
    // given
    memset(serialConfig(), 0, sizeof(*serialConfig()));

    // when
    serialInit(true);

    EXPECT_EQ(SERIAL_PORT_COUNT, 8);
    EXPECT_EQ(SERIAL_PORT_COUNT, serialPortCount);
}

TEST(IoSerialTest, TestSoftSerialPortsDisabled)
{
    // given
    memset(serialConfig(), 0, sizeof(*serialConfig()));

    // when
    serialInit(false);

    EXPECT_EQ(SERIAL_PORT_COUNT - 2, serialPortCount);
}

TEST(IoSerialTest, TestFindPortConfig)
{
    // given
    memset(serialConfig(), 0, sizeof(*serialConfig()));

    // when
    serialInit(true);

    // and
    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP_SERVER);

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
void mspSerialProcess(void) {}
void systemResetToBootloader(void) {}

serialPort_t *usbVcpOpen(void) { return NULL; }
serialPort_t *uartOpen(USART_TypeDef *, serialReceiveCallbackPtr, uint32_t, portMode_t, portOptions_t) { return NULL; }
serialPort_t *openSoftSerial(softSerialPortIndex_e, serialReceiveCallbackPtr, uint32_t, portOptions_t) { return NULL; }
void serialSetMode(serialPort_t *, portMode_t) {}
}
