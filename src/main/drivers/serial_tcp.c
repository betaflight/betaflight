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
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"
#include "common/maths.h"

#include "io/serial.h"
#include "serial_tcp.h"

#define BASE_PORT 5760

static const struct serialPortVTable tcpVTable; // forward declaration, defined at end of file
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
    if(s->clientCount == 0) {
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

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    tcpPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
	s = tcpReconfigure(&tcpSerialPorts[id], id);
    }
#endif
    if (!s)
        return NULL;

    serialImplOpen(&s->port, mode, &tcpVTable,
                   s->rxBuffer, sizeof(s->rxBuffer), s->txBuffer, sizeof(s->txBuffer));

    s->port.rxCallback = rxCallback;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return &s->port;
}

void tcpKickTx(serialPort_t *instance)
{
    tcpPort_t *s = container_of(instance, tcpPort_t, port);
    UNUSED(s);
    // NOP now, dyad will check periodically
}

static void tcpEndWrite(serialPort_t *instance)
{
    tcpPort_t *s = container_of(instance, tcpPort_t, port);
    tcpDataOut(s);  // force flush of buffer
}

// bridge to dyad functions to handle data
void tcpDataOut(tcpPort_t *s)
{
    if (s->conn == NULL)
        return;

    void *txData;
    const int txLen = serialGetTxData(&s->port, &txData);
    if (txLen > 0) {
#if TODO
        const int written = dyad_write(s->conn, txData, txLen);
        serialAckTxData(&s->port, written);
#else
        dyad_write(s->conn, txData, txLen);
        serialAckTxData(&s->port, txLen);
#endif
    }
}

void tcpDataIn(tcpPort_t *s, const void* data, int size)
{
    while (size > 0) {
        void *rxBuff;
        int available = serialGetRxDataBuffer(&s->port, &rxBuff);
        if (available < 0) {
            // no space to store data, discard them
            fprintf(stderr, "UART%u: %d bytes of data discarded\n", s->id + 1, size);
            return;
        }
        const int chunk = MIN(available, size);
        memcpy(rxBuff, data, chunk);
        data = (uint8_t*)data + chunk; size -= chunk;
        serialAckRxData(&s->port, chunk);
    }
}

static const struct serialPortVTable tcpVTable = {
    SERIALIMPL_VTABLE,
    .serialSetBaudRate = NULL,
    .setMode = NULL,
    .beginWrite = NULL,
    .endWrite = tcpEndWrite,
    .kickTx = tcpKickTx,
};
