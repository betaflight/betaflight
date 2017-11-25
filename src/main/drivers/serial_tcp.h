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

#pragma once

#include <netinet/in.h>
#include <pthread.h>
#include "dyad.h"

#define RX_BUFFER_SIZE    1400
#define TX_BUFFER_SIZE    1400

typedef struct {
    serialPort_t port;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint8_t txBuffer[TX_BUFFER_SIZE];

    dyad_Stream *serv;
    dyad_Stream *conn;
    pthread_mutex_t txLock;
    pthread_mutex_t rxLock;
    bool connected;
    uint16_t clientCount;
    uint8_t id;
} tcpPort_t;

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);

// tcpPort API
void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size);
void tcpDataOut(tcpPort_t *instance);

bool tcpIsStart(void);
bool* tcpGetUsed(void);
tcpPort_t* tcpGetPool(void);
