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

//#pragma once

#include "platform.h"

#include "drivers/timer.h"

typedef uint16_t dmaCode_t;

typedef struct dmaChannelSpec_s {
    dmaCode_t             code;
#if defined(STM32F4) || defined(STM32F7)
    DMA_Stream_TypeDef    *ref;
    uint32_t              channel;
#else
    DMA_Channel_TypeDef   *ref;
#endif
} dmaChannelSpec_t;

#define DMA_CODE(dma, stream, chanreq) ((dma << 12)|(stream << 8)|(chanreq << 0))
#define DMA_CODE_CONTROLLER(code) ((code >> 12) & 0xf)
#define DMA_CODE_STREAM(code) ((code >> 8) & 0xf)
#define DMA_CODE_CHANNEL(code) ((code >> 0) & 0xff)
#define DMA_CODE_REQUEST(code) DMA_CODE_CHANNEL(code)

typedef enum {
    DMA_PERIPH_SPI_TX,
    DMA_PERIPH_SPI_RX,
    DMA_PERIPH_ADC,
    DMA_PERIPH_SDIO,
    DMA_PERIPH_UART_TX,
    DMA_PERIPH_UART_RX,
} dmaPeripheral_e;

typedef int8_t dmaoptValue_t;

#define DMA_OPT_UNUSED (-1)

#define MAX_PERIPHERAL_DMA_OPTIONS 2
#define MAX_TIMER_DMA_OPTIONS 3

dmaoptValue_t dmaoptByTag(ioTag_t ioTag);
const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt);
const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt);
const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer);
dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer);
