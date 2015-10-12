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

#define SOFTSERIAL_BUFFER_SIZE 256

typedef enum {
    SOFTSERIAL1 = 0,
    SOFTSERIAL2
} softSerialPortIndex_e;

serialPort_t *openSoftSerial(softSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, uint32_t baud, portOptions_t options);

// serialPort API
void softSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t softSerialRxBytesWaiting(serialPort_t *instance);
uint8_t softSerialTxBytesFree(serialPort_t *instance);
uint8_t softSerialReadByte(serialPort_t *instance);
void softSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isSoftSerialTransmitBufferEmpty(serialPort_t *s);

