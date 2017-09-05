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

#include "build/debug.h"

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
    ++instance->statTxBytes;
}


void serialWriteBuf(serialPort_t *instance, const uint8_t *data, int count)
{
    if (instance->vTable->writeBuf) {
        instance->vTable->writeBuf(instance, data, count);
    } else {
        for (const uint8_t *p = data; count > 0; count--, p++) {

            while (!serialTxBytesFree(instance)) {
            };

            serialWrite(instance, *p);
        }
    }
    instance->statTxBytes += count;
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
    ++instance->statRxBytes;
    return instance->vTable->serialRead(instance);
}

void serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetMode(serialPort_t *instance, portMode_e mode)
{
    instance->vTable->setMode(instance, mode);
}

void serialWriteBufShim(void *instance, const uint8_t *data, int count)
{
    serialWriteBuf((serialPort_t *)instance, data, count);
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

// Statistics

void serialStatSetState(serialPort_t *instance, int state)
{
    if (instance) {
        instance->statState = state;
    }
}

void serialStatRxFrame(serialPort_t *instance)
{
    if (instance) {
        ++instance->statRxFrames;
    }
}

void serialStatTxFrame(serialPort_t *instance)
{
    if (instance) {
        ++instance->statTxFrames;
    }
}

void serialStatError(serialPort_t *instance)
{
    if (instance) {
        ++instance->statErrors;
    }
}

void serialStatCopyToDebug(serialPort_t *instance)
{
    static uint32_t count = 0;

    if (instance) {
        debug[0] = instance->statState * 1000 + (++count % 1000);
        debug[1] = (instance->statRxBytes % 100) * 100 + (instance->statTxBytes % 100);
        debug[2] = (instance->statRxFrames % 100) * 100 + (instance->statTxFrames % 100);
        debug[3] = instance->statErrors;
    }
}
