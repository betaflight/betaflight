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
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "platform.h"

#ifdef USE_SERIAL_BRIDGE

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial.h"
#include "serial_bridge.h"

#define RX_BUFFER_SIZE    512
#define TX_BUFFER_SIZE    512

typedef struct {
    serialPort_t port;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t txBuffer[TX_BUFFER_SIZE];

    bool connected;                         // needs to be in the olrs stack, not here.

} sbrPort_t;

static sbrPort_t sbrPort;


static serialPort_t *portFrom = NULL;
static serialPort_t *portTo = NULL;

// private

static uint32_t getTotalRxBytesWaiting(const serialPort_t *instance) {
    sbrPort_t *s = (sbrPort_t*) instance;
    uint32_t count;

    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }

    return count;
}

static uint32_t getTotalTxBytesFree(const serialPort_t *instance) {
    sbrPort_t *s = (sbrPort_t*) instance;
    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;

    return bytesFree;
}

static bool isTransmitBufferEmpty(const serialPort_t *instance) {
    sbrPort_t *s = (sbrPort_t *) instance;

    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;

    return isEmpty;
}

static uint8_t sbrRead(serialPort_t *instance) {
    uint8_t ch;
    sbrPort_t *s = (sbrPort_t *) instance;

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    return ch;
}

static void sbrWrite(serialPort_t *instance, uint8_t ch) {
    sbrPort_t *s = (sbrPort_t *) instance;

//    if (!s->connected)
//        return;

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }
}

static const struct serialPortVTable sbrVTable = {
    .serialWrite = sbrWrite,
    .serialTotalRxWaiting = getTotalRxBytesWaiting,
    .serialTotalTxFree = getTotalTxBytesFree,
    .serialRead = sbrRead,
    .serialSetBaudRate = NULL,
    .isSerialTransmitBufferEmpty = isTransmitBufferEmpty,
    .setMode = NULL,
    .writeBuf = NULL,
    .beginWrite = NULL,
    .endWrite = NULL,
};



// public

// get a chunk of data from the vcp serial buffer for the olrs stack
int serialbridge_getChunk(uint8_t* ch, int size) {
    sbrPort_t *s = &sbrPort;
    int num = 0;

    while (size-- && (s->port.txBufferTail != s->port.txBufferHead)) {
        *ch++ = s->port.txBuffer[s->port.txBufferTail];
        if (s->port.txBufferTail + 1 >= s->port.txBufferSize) {
            s->port.txBufferTail = 0;
        } else {
            s->port.txBufferTail++;
        }
        num++;
    }
    return num;
}

// write a chunk of data into the vcp serial buffer from the olrs stack
void serialBridge_setChunk(uint8_t* ch, int size) {
    sbrPort_t *s = &sbrPort;

    while (size--) {

        if (s->port.rxCallback == NULL) {

            s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
            if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
                s->port.rxBufferHead = 0;
            } else {
                s->port.rxBufferHead++;
            }
        } else {
            s->port.rxCallback(*(ch++), NULL);
        }
    }
}


void serialBridgeInit(void) {
    if ((portFrom == NULL) && (portTo == NULL)){
        serialPortConfig_t *portConfig;
        portConfig = findSerialPortConfig(FUNCTION_SERIAL_BRIDGE);
        if (portConfig != NULL) {
            portFrom = openSerialPort(portConfig->identifier, FUNCTION_SERIAL_BRIDGE, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        }

        portConfig = findNextSerialPortConfig(FUNCTION_SERIAL_BRIDGE);
        if (portConfig != NULL) {
            portTo = openSerialPort(portConfig->identifier, FUNCTION_SERIAL_BRIDGE, NULL, NULL, baudRates[portConfig->msp_baudrateIndex], MODE_RXTX, SERIAL_NOT_INVERTED);
        }
    }

}

serialPort_t *serialBridgeOpen(serialReceiveCallbackPtr rxCallback) {
    sbrPort_t *s = &sbrPort;

    s->port.vTable = &sbrVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    s->port.rxCallback = rxCallback;

    return (serialPort_t *) s;
}

void serialBridgeProcess(void) {
    if ((portFrom != NULL) && (portTo != NULL)) {
        while (serialRxBytesWaiting(portFrom)) {
            const uint8_t c = serialRead(portFrom);
            serialWrite(portTo, c);
        }

        while (serialRxBytesWaiting(portTo)) {
            const uint8_t c = serialRead(portTo);
            serialWrite(portFrom, c);
        }
    }
}

#endif
