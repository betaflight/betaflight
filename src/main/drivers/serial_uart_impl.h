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

// device specific uart implementation is defined here

extern const struct serialPortVTable uartVTable[];

void uartStartTxDMA(uartPort_t *s);

uartPort_t *serialUSART1(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUSART2(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUSART3(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUSART4(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUSART5(uint32_t baudRate, portMode_t mode, portOptions_t options);
uartPort_t *serialUSART6(uint32_t baudRate, portMode_t mode, portOptions_t options);

