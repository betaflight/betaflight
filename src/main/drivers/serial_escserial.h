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

#define ESCSERIAL_BUFFER_SIZE 1024

typedef enum {
    ESCSERIAL1 = 0,
    ESCSERIAL2
} escSerialPortIndex_e;

serialPort_t *openEscSerial(escSerialPortIndex_e portIndex, serialReceiveCallbackPtr callback, uint16_t output, uint32_t baud, portOptions_t options, uint8_t mode);

// serialPort API
void escSerialWriteByte(serialPort_t *instance, uint8_t ch);
uint8_t escSerialTotalBytesWaiting(serialPort_t *instance);
uint8_t escSerialReadByte(serialPort_t *instance);
void escSerialSetBaudRate(serialPort_t *s, uint32_t baudRate);
bool isEscSerialTransmitBufferEmpty(serialPort_t *s);
void escSerialInitialize();
void escEnablePassthrough(serialPort_t *escPassthroughPort, uint16_t output, uint8_t mode);
