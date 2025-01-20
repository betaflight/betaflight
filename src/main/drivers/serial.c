/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "io/serial.h"
#include "serial.h"

void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

uint32_t serialGetBaudRate(serialPort_t *instance)
{
    return instance->baudRate;
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

void serialWriteBufNoFlush(serialPort_t *instance, const uint8_t *data, int count)
{
    if (instance->vTable->writeBuf) {
        instance->vTable->writeBuf(instance, data, count);
    } else {
        // The transmit buffer is large enough to hold any single message, so only wait once
        while (serialTxBytesFree(instance) < (uint32_t)count) {
        };

        for (const uint8_t *p = data; count > 0; count--, p++) {
            serialWrite(instance, *p);
        }
    }
}

uint32_t serialRxBytesWaiting(const serialPort_t *instance)
{
    return instance->vTable->serialTotalRxWaiting(instance);
}

uint32_t serialTxBytesFree(const serialPort_t *instance)
{
    return instance->vTable->serialTotalTxFree(instance);
}

uint8_t serialRead(serialPort_t *instance)
{
    return instance->vTable->serialRead(instance);
}

void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    //vTable->serialSetBaudRate is NULL for SIMULATOR_BUILD, because the TCP port is used
    if (instance->vTable->serialSetBaudRate != NULL) {
        instance->vTable->serialSetBaudRate(instance, baudRate);
    }
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetMode(serialPort_t *instance, portMode_e mode)
{
    instance->vTable->setMode(instance, mode);
}

void serialSetCtrlLineStateCb(serialPort_t *serialPort, void (*cb)(void *context, uint16_t ctrlLineState), void *context)
{
    // If a callback routine for changes to control line state is supported by the underlying
    // driver, then set the callback.
    if (serialPort->vTable->setCtrlLineStateCb) {
        serialPort->vTable->setCtrlLineStateCb(serialPort, cb, context);
    }
}

void serialSetBaudRateCb(serialPort_t *serialPort, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context)
{
    // If a callback routine for changes to baud rate is supported by the underlying
    // driver, then set the callback.
    if (serialPort->vTable->setBaudRateCb) {
        serialPort->vTable->setBaudRateCb(serialPort, cb, context);
    }
}

void serialBeginWrite(serialPort_t *instance)
{
    if (instance->vTable->beginWrite)
        instance->vTable->beginWrite(instance);
}

void serialEndWrite(serialPort_t *instance)
{
    if (instance->vTable->endWrite)
        instance->vTable->endWrite(instance);
}

void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    serialBeginWrite(instance);
    serialWriteBufNoFlush(instance, data, count);
    serialEndWrite(instance);
}

void serialWriteBufShim(void *instance, const uint8_t *data, int count)
{
    serialWriteBuf((serialPort_t *)instance, data, count);
}

