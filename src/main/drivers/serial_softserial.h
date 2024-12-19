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

#pragma once

#define SOFTSERIAL_BUFFER_SIZE 256

#include "io/serial.h"

serialPort_t *softSerialOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baud, portMode_e mode, portOptions_e options);

// serialPort API
void softSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint32_t softSerialRxBytesWaiting(const serialPort_t *instance);
uint32_t softSerialTxBytesFree(const serialPort_t *instance);
uint8_t softSerialReadByte(serialPort_t *instance);
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isSoftSerialTransmitBufferEmpty(const serialPort_t *s);
