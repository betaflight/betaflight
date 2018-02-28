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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/serial.h"

#include "serial_rtt.h"
#include "debug.h"

static void rttSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    // Ignore
    UNUSED(instance);
    UNUSED(baudRate);
}

static void rttSetMode(serialPort_t *instance, portMode_e mode)
{
    // Ignore
    UNUSED(instance);
    UNUSED(mode);
}

static uint32_t rttTotalRxBytesWaiting(const serialPort_t *instance)
{
    // Ignore
    UNUSED(instance);

    return SEGGER_RTT_HasData(RTT_SERIAL_CHANNEL);
}

static uint32_t rttTotalTxBytesFree(const serialPort_t *instance)
{
    // Ignore
    UNUSED(instance);

    // Fake this. The serial (or debug) channels overflow behavior is independent of this
    return 80;
}

static bool isRttTransmitBufferEmpty(const serialPort_t *instance)
{
    // Ignore
    UNUSED(instance);

    // Always return true
    return true;
}

static uint8_t rttRead(serialPort_t *instance)
{
    // Ignore
    UNUSED(instance);

    uint8_t ch;

    (void)SEGGER_RTT_Read(RTT_SERIAL_CHANNEL, &ch, sizeof (ch));

    return ch;
}

static void rttWrite(serialPort_t *instance, uint8_t ch)
{
    // Ignore
    UNUSED(instance);

    SEGGER_RTT_PutChar(RTT_SERIAL_CHANNEL, ch);
}

const struct serialPortVTable rttVTable[] = {
    {
        .serialWrite = rttWrite,
        .serialTotalRxWaiting = rttTotalRxBytesWaiting,
        .serialTotalTxFree = rttTotalTxBytesFree,
        .serialRead = rttRead,
        .serialSetBaudRate = rttSetBaudRate,
        .isSerialTransmitBufferEmpty = isRttTransmitBufferEmpty,
        .setMode = rttSetMode,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
    }
};

serialPort_t *openRttSerial()
{
    static serialPort_t instance;

#if (RTT_SERIAL_CHANNEL != RTT_DEBUG_CHANNEL)
#if (RTT_DEBUG_CHANNEL == 0)
    SEGGER_RTT_ConfigUpBuffer(RTT_SERIAL_CHANNEL, "RTT Serial", NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
#else
    static char dbgBuf[BUFFER_SIZE_UP];
    SEGGER_RTT_ConfigUpBuffer(RTT_SERIAL_CHANNEL, "RTT Serial", dbgBuf, BUFFER_SIZE_UP, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
#endif
#endif

    // Reference the table of serial driver functions
    instance.vTable = rttVTable;

    return &instance;
}

