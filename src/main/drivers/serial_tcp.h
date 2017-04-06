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
// Since serial ports can be used for any function these buffer sizes should be equal
// The two largest things that need to be sent are: 1, MSP responses, 2, UBLOX SVINFO packet.

// Size must be a power of two due to various optimizations which use 'and' instead of 'mod'
// Various serial routines return the buffer occupied size as uint8_t which would need to be extended in order to
// increase size further.
#define RX_BUFFER_SIZE    1400
#define TX_BUFFER_SIZE    1400

typedef struct {
    serialPort_t port;
	volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
	volatile uint8_t txBuffer[TX_BUFFER_SIZE];

	dyad_Stream *serv;
	dyad_Stream *conn;
	pthread_mutex_t txLock;
	pthread_mutex_t rxLock;
	bool connected;
	uint16_t clientCount;
	uint8_t id;
} uartPort_t;

serialPort_t *uartOpen(USART_TypeDef *USARTx, serialReceiveCallbackPtr rxCallback, uint32_t baudRate, portMode_t mode, portOptions_t options);

// serialPort API
void tcpWrite(serialPort_t *instance, uint8_t ch);
void tcpDataIn(uartPort_t *instance, uint8_t* ch, int size);
uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance);
uint32_t tcpTotalTxBytesFree(const serialPort_t *instance);
uint8_t tcpRead(serialPort_t *instance);
void tcpDataOut(uartPort_t *instance);
bool isTcpTransmitBufferEmpty(const serialPort_t *s);

bool tcpIsStart(void);
bool* tcpGetUsed(void);
uartPort_t* tcpGetPool(void);

