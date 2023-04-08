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

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_tcp.h"

#define BASE_PORT 5760

static const struct serialPortVTable tcpVTable; // Forward
static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
static bool tcpPortInitialized[SERIAL_PORT_COUNT];
static bool tcpStart = false;
bool tcpIsStart(void)
{
    return tcpStart;
}
static void onData(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    tcpDataIn(s, (uint8_t*)e->data, e->size);
}
static void onClose(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    s->clientCount--;
    s->conn = NULL;
    fprintf(stderr, "[CLS]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    if (s->clientCount == 0) {
        s->connected = false;
    }
}
static void onAccept(dyad_Event *e)
{
    tcpPort_t* s = (tcpPort_t*)(e->udata);
    fprintf(stderr, "New connection on UART%u, %d\n", s->id + 1, s->clientCount);

    s->connected = true;
    if (s->clientCount > 0) {
        dyad_close(e->remote);
        return;
    }
    s->clientCount++;
    fprintf(stderr, "[NEW]UART%u: %d,%d\n", s->id + 1, s->connected, s->clientCount);
    s->conn = e->remote;
    dyad_setNoDelay(e->remote, 1);
    dyad_setTimeout(e->remote, 120);
    dyad_addListener(e->remote, DYAD_EVENT_DATA, onData, e->udata);
    dyad_addListener(e->remote, DYAD_EVENT_CLOSE, onClose, e->udata);
}
static tcpPort_t* tcpReconfigure(tcpPort_t *s, int id)
{
    if (tcpPortInitialized[id]) {
        fprintf(stderr, "port is already initialized!\n");
        return s;
    }

    if (pthread_mutex_init(&s->txLock, NULL) != 0) {
        fprintf(stderr, "TX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        // TODO: clean up & re-init
        return NULL;
    }

    tcpStart = true;
    tcpPortInitialized[id] = true;

    s->connected = false;
    s->clientCount = 0;
    s->id = id;
    s->conn = NULL;
    s->serv = dyad_newStream();
    dyad_setNoDelay(s->serv, 1);
    dyad_addListener(s->serv, DYAD_EVENT_ACCEPT, onAccept, s);

    if (dyad_listenEx(s->serv, NULL, BASE_PORT + id + 1, 10) == 0) {
        fprintf(stderr, "bind port %u for UART%u\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    } else {
        fprintf(stderr, "bind port %u for UART%u failed!!\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
    }
    return s;
}

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    tcpPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
    s = tcpReconfigure(&tcpSerialPorts[id], id);
    }
#endif
    if (!s)
        return NULL;

    s->port.vTable = &tcpVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t count;
    pthread_mutex_lock(&s->rxLock);
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
    pthread_mutex_unlock(&s->rxLock);

    return count;
}

uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t bytesUsed;

    pthread_mutex_lock(&s->txLock);
    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;
    pthread_mutex_unlock(&s->txLock);

    return bytesFree;
}

bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);
    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;
    pthread_mutex_unlock(&s->txLock);
    return isEmpty;
}

uint8_t tcpRead(serialPort_t *instance)
{
    uint8_t ch;
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    pthread_mutex_unlock(&s->rxLock);

    return ch;
}

void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->txLock);

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }
    pthread_mutex_unlock(&s->txLock);

    tcpDataOut(s);
}

void tcpDataOut(tcpPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    if (s->conn == NULL) return;
    pthread_mutex_lock(&s->txLock);

    if (s->port.txBufferHead < s->port.txBufferTail) {
        // send data till end of buffer
        int chunk = s->port.txBufferSize - s->port.txBufferTail;
        dyad_write(s->conn, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        s->port.txBufferTail = 0;
    }
    int chunk = s->port.txBufferHead - s->port.txBufferTail;
    if (chunk)
        dyad_write(s->conn, (const void*)&s->port.txBuffer[s->port.txBufferTail], chunk);
    s->port.txBufferTail = s->port.txBufferHead;

    pthread_mutex_unlock(&s->txLock);
}

void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    while (size--) {
//        printf("%c", *ch);
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->rxLock);
//    printf("\n");
}

static const struct serialPortVTable tcpVTable = {
        .serialWrite = tcpWrite,
        .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
        .serialTotalTxFree = tcpTotalTxBytesFree,
        .serialRead = tcpRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
};
