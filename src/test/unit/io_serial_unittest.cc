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
    #include "drivers/serial_softserial.h"
    #include "drivers/serial_uart.h"

    #include "io/serial.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    void serialInit(bool softserialEnabled, serialPortIdentifier_e serialPortToDisable);

    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(IoSerialTest, TestFindPortConfig)
{
    // given
    serialInit(false, SERIAL_PORT_NONE);

    // when
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_MSP);

    // then
    EXPECT_EQ(NULL, portConfig);
}


// STUBS
extern "C" {
    void delay(uint32_t) {}

    bool isSerialTransmitBufferEmpty(const serialPort_t *) { return true; }

    void systemResetToBootloader(void) {}

    bool telemetryCheckRxPortShared(const serialPortConfig_t *) { return false; }

    uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
    uint8_t serialRead(serialPort_t *) { return 0; }
    void serialWrite(serialPort_t *, uint8_t) {}

    serialPort_t *usbVcpOpen(void) { return NULL; }

    serialPort_t *uartOpen(serialPortIdentifier_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {
      return NULL;
    }

    serialPort_t *softSerialOpen(serialPortIdentifier_e, serialReceiveCallbackPtr, void *, uint32_t, portMode_e, portOptions_e) {
      return NULL;
    }

    void serialSetCtrlLineStateCb(serialPort_t *, void (*)(void *, uint16_t ), void *) {}
    void serialSetCtrlLineState(serialPort_t *, uint16_t ) {}
    uint32_t serialTxBytesFree(const serialPort_t *) {return 1;}

    void serialSetBaudRateCb(serialPort_t *, void (*)(serialPort_t *context, uint32_t baud), serialPort_t *) {}

    void pinioSet(int, bool) {}
}
