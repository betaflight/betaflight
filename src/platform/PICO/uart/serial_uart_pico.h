/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "hardware/pio.h"

typedef struct pioUartHardware_s {
    serialPortIdentifier_e identifier;
    uint8_t irqn;
    volatile uint8_t *txBuffer;
    volatile uint8_t *rxBuffer;
    uint16_t txBufferSize;
    uint16_t rxBufferSize;
} pioUartHardware_t;

void uartPinConfigure_pio(const serialPinConfig_t *pSerialPinConfig);
bool serialUART_pio(uartPort_t *s, uint32_t baudRate, portMode_e mode, portOptions_e options,
                    const pioUartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO);
void uartReconfigure_pio(uartPort_t *s);
void uartEnableTxInterrupt_pio(uartPort_t *uartPort);


void uartPinConfigure_hw(const serialPinConfig_t *pSerialPinConfig);
bool serialUART_hw(uartPort_t *s, uint32_t baudRate, portMode_e mode, portOptions_e options,
                   const uartHardware_t *hardware, serialPortIdentifier_e identifier, IO_t txIO, IO_t rxIO);
void uartReconfigure_hw(uartPort_t *s);
void uartEnableTxInterrupt_hw(uartPort_t *uartPort);
