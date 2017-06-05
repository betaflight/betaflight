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
#include <string.h>

#include "platform.h"

#include "common/maths.h"

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
    instance->vTable->serialSetBaudRate(instance, baudRate);
}

bool isSerialTransmitBufferEmpty(const serialPort_t *instance)
{
    return instance->vTable->isSerialTransmitBufferEmpty(instance);
}

void serialSetMode(serialPort_t *instance, portMode_t mode)
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

// common implementation of serial buffers. Device driver can access receive and transmit buffer as memory blocks

bool serialImplOpen(serialPort_t* port, portMode_t mode, const struct serialPortVTable *vtable,
                    void* rxBuffer, int rxSize, void* txBuffer, int txSize,
                    serialReceiveCallbackPtr rxCallback)
{
    port->vTable       = vtable;
    port->mode         = mode;

    port->rxBuffer     = rxBuffer;
    port->rxBufferSize = rxSize;
    port->rxBufferTail = 0;
    port->rxBufferHead = 0;

    port->txBuffer     = txBuffer;
    port->txBufferSize = txSize;
    port->txBufferTail = 0;
    port->txBufferHead = 0;

    port->rxCallback = rxCallback;

    return true;
}

static void serialKickTx(serialPort_t *port)
{
    port->vTable->kickTx(port);
}

void serialImplWrite(serialPort_t *port, uint8_t ch)
{
    uint32_t nxt = (port->txBufferHead + 1 >= port->txBufferSize) ? 0 : port->txBufferHead + 1;
    if(nxt == port->txBufferTail) {
        // buffer full. Discard byte now
    } else {
        port->txBuffer[port->txBufferHead] = ch;
        port->txBufferHead = nxt;
        serialKickTx(port);
    }
}

void serialImplWriteBuf(serialPort_t *port, const void *data, int size)
{
    if (port->txBufferHead >= port->txBufferTail) { // free space from txBufferHeadand to txBufferSize
        const int chunk = port->txBufferSize - port->txBufferHead;
        if (size < chunk) { // all data fit, no wrap
            memcpy(&port->txBuffer[port->txBufferHead], data, size);
            port->txBufferHead += size;
            serialKickTx(port);
            return;
        }
        // space to end of buffer filled completely, wrap head
        memcpy(&port->txBuffer[port->txBufferHead], data, chunk);
        port->txBufferHead = 0;
        data = ((uint8_t*)data + chunk); size -= chunk;
    }
    // space from txBufferHead to txBufferTail-1
    const int chunk = MIN(size, (int)(port->txBufferTail - port->txBufferHead - 1));
    memcpy(&port->txBuffer[port->txBufferHead], data, chunk);
    port->txBufferHead += chunk;
    serialKickTx(port);
}

uint8_t serialImplRead(serialPort_t *port)
{
    if (port->rxBufferHead == port->rxBufferTail) {
        return -1;
    }

    int ch = port->rxBuffer[port->rxBufferTail];
    port->rxBufferTail = (port->rxBufferTail + 1 >= port->rxBufferSize) ? 0 : port->rxBufferTail + 1;
    return ch;
}

bool serialImplIsTransmitBufferEmpty(const serialPort_t *port)
{
    return port->txBufferHead == port->txBufferTail;
}

uint32_t serialImplTxBytesFree(const serialPort_t *port)
{
    if ((port->mode & MODE_TX) == 0) {
        return 0;
    }

    int bytesUsed = (port->txBufferHead - port->txBufferTail);
    if(bytesUsed < 0)
        bytesUsed += port->txBufferSize;

    return (port->txBufferSize - 1) - bytesUsed;
}

uint32_t serialImplRxBytesWaiting(const serialPort_t *port)
{
    int ret = port->rxBufferHead - port->rxBufferTail;
    if(ret < 0)
        ret += port->rxBufferSize;
    return ret;
}

// callback to get data to be transmitted
// return positive length when thera are available data
// actual length used will be passed back using Ack function
int serialGetTxData(serialPort_t *port, void **dataPtr)
{
    *dataPtr = port->txBuffer + port->txBufferTail;
    if (port->txBufferHead >= port->txBufferTail) {
        return port->txBufferHead - port->txBufferTail;
    } else {
        return port->txBufferSize - port->txBufferTail;
    }
}

// acknowledge data actually transmitted
// called after data has been copied from buffer / processed
// txLen must be smaller or equal to size returned by serialGetTxData
void serialAckTxData(serialPort_t *port, int txLen)
{
    unsigned nxt = port->txBufferTail + txLen;
    if(nxt >= port->txBufferSize)
        nxt = 0;
    port->txBufferTail = nxt;
}

// get buffer for received data and its length
// returns 0 if there is no space in buffer (caller should discard data)
// serialAckRxData must be called after data are stored into bufer, passing actually used buffer size
int serialGetRxDataBuffer(serialPort_t *port, void **dataPtr)
{
    *dataPtr = port->rxBuffer + port->rxBufferHead;
    if (port->rxBufferHead >= port->rxBufferTail) {
        // space up to end of buffer
        return port->rxBufferSize - port->rxBufferHead;
    } else {
        // space to tail - 1
        return port->rxBufferTail - 1 - port->rxBufferHead;
    }
}

// commit data stored into buffer returned by serialGetRxDataBuffer
// len must be less or equal to size returned by serialGetRxDataBuffer
void serialAckRxData(serialPort_t *port, int len)
{
    // TODO - rxcallback is far too intrusive
    if(port->rxCallback)
        for(unsigned i = port->rxBufferHead; i < port->rxBufferHead + len; i++)
            port->rxCallback(port->rxBuffer[i]);

    unsigned nxt = port->rxBufferHead + len;
    if(nxt >= port->rxBufferSize)
        nxt = 0;
    port->rxBufferHead = nxt;
}



