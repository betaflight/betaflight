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

#include "drivers/dma.h" // For dmaResource_t
#include "io/serial.h"   // TODO: maybe move serialPortIdentifier_e into separate header

typedef struct uartPort_s {
    serialPort_t port;

#ifdef USE_DMA
#ifdef USE_HAL_DRIVER
    DMA_HandleTypeDef rxDMAHandle;
    DMA_HandleTypeDef txDMAHandle;
#endif

    dmaResource_t *rxDMAResource;
    dmaResource_t *txDMAResource;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;
#if defined(USE_ATBSP_DRIVER)
    uint32_t rxDMAMuxId;
    uint32_t txDMAMuxId;
#endif

    uint32_t rxDMAIrq;
    uint32_t txDMAIrq;

    uint32_t rxDMAPos;

    uint32_t txDMAPeripheralBaseAddr;
    uint32_t rxDMAPeripheralBaseAddr;
#endif // USE_DMA

#ifdef USE_HAL_DRIVER
    // All USARTs can also be used as UART, and we use them only as UART.
    UART_HandleTypeDef Handle;
#endif
    USART_TypeDef *USARTx;
    bool txDMAEmpty;

    bool (* checkUsartTxOutput)(struct uartPort_s *s);
} uartPort_t;

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig);
serialPort_t *uartOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);
