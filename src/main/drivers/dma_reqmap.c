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

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "drivers/adc.h"
#include "drivers/bus_spi.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/timer_def.h"

#include "pg/timerio.h"

#include "dma_reqmap.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
} dmaTimerMapping_t;

#if defined(STM32F4) || defined(STM32F7)

#if defined(STM32F4)
#define DMA(d, s, c) { DMA_CODE(d, s, c), DMA ## d ## _Stream ## s, DMA_Channel_ ## c }
#elif defined(STM32F7)
#define DMA(d, s, c) { DMA_CODE(d, s, c), DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }
#endif

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    // Everything including F405 and F446
    { DMA_PERIPH_SPI_TX,  SPIDEV_1,  { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_1,  { DMA(2, 0, 3), DMA(2, 2, 3) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_2,  { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_2,  { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_TX,  SPIDEV_3,  { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_3,  { DMA(1, 0, 0), DMA(1, 2, 0) } },

#if defined(STM32F411xE) || defined(STM32F745xx) || defined(STM32F746xx) || defined(STM32F765xx) || defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_4,  { DMA(2, 1, 4) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_4,  { DMA(2, 0, 4) } },

#ifdef USE_EXTENDED_SPI_DEVICE
    { DMA_PERIPH_SPI_TX,  SPIDEV_5,  { DMA(2, 6, 7) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_5,  { DMA(2, 5, 7) } },

#if !defined(STM32F722xx)
    { DMA_PERIPH_SPI_TX,  SPIDEV_6,  { DMA(2, 5, 1) } },
    { DMA_PERIPH_SPI_RX,  SPIDEV_6,  { DMA(2, 6, 1) } },
#endif
#endif // USE_EXTENDED_SPI_DEVICE
#endif
#endif // USE_SPI

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(2, 0, 0), DMA(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(2, 2, 1), DMA(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { DMA(2, 0, 2), DMA(2, 1, 2) } },
#endif

#ifdef USE_SDCARD_SDIO
    { DMA_PERIPH_SDIO,    0,         { DMA(2, 3, 4), DMA(2, 6, 4) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(2, 5, 4), DMA(2, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 5, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 1, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(1, 2, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 0, 4) } },
    { DMA_PERIPH_UART_TX, UARTDEV_6, { DMA(2, 6, 5), DMA(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { DMA(2, 1, 5), DMA(2, 2, 5) } },
#endif
};

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TIM1, TC(CH1), { DMA(2, 6, 0), DMA(2, 1, 6), DMA(2, 3, 6) } },
    { TIM1, TC(CH2), { DMA(2, 6, 0), DMA(2, 2, 6) } },
    { TIM1, TC(CH3), { DMA(2, 6, 0), DMA(2, 6, 6) } },
    { TIM1, TC(CH4), { DMA(2, 4, 6) } },

    { TIM2, TC(CH1), { DMA(1, 5, 3) } },
    { TIM2, TC(CH2), { DMA(1, 6, 3) } },
    { TIM2, TC(CH3), { DMA(1, 1, 3) } },
    { TIM2, TC(CH4), { DMA(1, 7, 3), DMA(1, 6, 3) } },

    { TIM3, TC(CH1), { DMA(1, 4, 5) } },
    { TIM3, TC(CH2), { DMA(1, 5, 5) } },
    { TIM3, TC(CH3), { DMA(1, 7, 5) } },
    { TIM3, TC(CH4), { DMA(1, 2, 5) } },

    { TIM4, TC(CH1), { DMA(1, 0, 2) } },
    { TIM4, TC(CH2), { DMA(1, 3, 2) } },
    { TIM4, TC(CH3), { DMA(1, 7, 2) } },

    { TIM5, TC(CH1), { DMA(1, 2, 6) } },
    { TIM5, TC(CH2), { DMA(1, 4, 6) } },
    { TIM5, TC(CH3), { DMA(1, 0, 6) } },
    { TIM5, TC(CH4), { DMA(1, 1, 6), DMA(1, 3, 6) } },

    { TIM8, TC(CH1), { DMA(2, 2, 0), DMA(2, 2, 7) } },
    { TIM8, TC(CH2), { DMA(2, 2, 0), DMA(2, 3, 7) } },
    { TIM8, TC(CH3), { DMA(2, 2, 0), DMA(2, 4, 7) } },
    { TIM8, TC(CH4), { DMA(2, 7, 7) } },
};
#undef TC
#undef DMA

#else // STM32F3
    // The embedded ADC24_DMA_REMAP conditional should be removed
    // when (and if) F3 is going generic.
#define DMA(d, c) { DMA_CODE(d, 0, c), DMA ## d ## _Channel ## c }
static const dmaPeripheralMapping_t dmaPeripheralMapping[17] = {
#ifdef USE_SPI
    { DMA_PERIPH_SPI_TX,  1, { DMA(1, 3) } },
    { DMA_PERIPH_SPI_RX,  1, { DMA(1, 2) } },
    { DMA_PERIPH_SPI_TX,  2, { DMA(1, 5) } },
    { DMA_PERIPH_SPI_RX,  2, { DMA(1, 4) } },
    { DMA_PERIPH_SPI_TX,  3, { DMA(2, 2) } },
    { DMA_PERIPH_SPI_RX,  3, { DMA(2, 1) } },
#endif

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     1, { DMA(1, 1) } },
#ifdef ADC24_DMA_REMAP
    { DMA_PERIPH_ADC,     2, { DMA(2, 3) } },
#else
    { DMA_PERIPH_ADC,     2, { DMA(2, 1) } },
#endif
    { DMA_PERIPH_ADC,     3, { DMA(2, 5) } },
#endif

#ifdef USE_UART
    { DMA_PERIPH_UART_TX, 1, { DMA(1, 4) } },
    { DMA_PERIPH_UART_RX, 1, { DMA(1, 5) } },

    { DMA_PERIPH_UART_TX, 2, { DMA(1, 7) } },
    { DMA_PERIPH_UART_RX, 2, { DMA(1, 6) } },
    { DMA_PERIPH_UART_TX, 3, { DMA(1, 2) } },
    { DMA_PERIPH_UART_RX, 3, { DMA(1, 3) } },
    { DMA_PERIPH_UART_TX, 4, { DMA(2, 5) } },
    { DMA_PERIPH_UART_RX, 4, { DMA(2, 3) } },
};
#endif

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TIM1, TC(CH1), { DMA(1, 2) } },
    { TIM1, TC(CH2), { DMA(1, 3) } },
    { TIM1, TC(CH3), { DMA(1, 6) } },
    { TIM1, TC(CH4), { DMA(1, 4) } },

    { TIM2, TC(CH1), { DMA(1, 5) } },
    { TIM2, TC(CH2), { DMA(1, 7) } },
    { TIM2, TC(CH3), { DMA(1, 1) } },
    { TIM2, TC(CH4), { DMA(1, 7) } },

    { TIM3, TC(CH1), { DMA(1, 6) } },
    { TIM3, TC(CH3), { DMA(1, 2) } },
    { TIM3, TC(CH4), { DMA(1, 3) } },

    { TIM4, TC(CH1), { DMA(1, 1) } },
    { TIM4, TC(CH2), { DMA(1, 4) } },
    { TIM4, TC(CH3), { DMA(1, 5) } },

    { TIM8, TC(CH1), { DMA(2, 3) } },
    { TIM8, TC(CH2), { DMA(2, 5) } },
    { TIM8, TC(CH3), { DMA(2, 1) } },
    { TIM8, TC(CH4), { DMA(2, 2) } },

    { TIM15, TC(CH1), { DMA(1, 5) } },

#ifdef REMAP_TIM16_DMA
    { TIM16, TC(CH1), { DMA(1, 6) } },
#else
    { TIM16, TC(CH1), { DMA(1, 3) } },
#endif

#ifdef REMAP_TIM17_DMA
    { TIM17, TC(CH1), { DMA(1, 7) } },
#else
    { TIM17, TC(CH1), { DMA(1, 1) } },
#endif
};

#undef TC
#undef DMA
#endif

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
    }

    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
#ifdef USE_TIMER_MGMT
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        if (timerIOConfig(i)->ioTag == ioTag) {
            return timerIOConfig(i)->dmaopt;
        }
    }
#else
    UNUSED(ioTag);
#endif

    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == tim && timerMapping->channel == channel && timerMapping->channelSpec[dmaopt].ref) {
            return &timerMapping->channelSpec[dmaopt];
        }
    }

    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }

    dmaoptValue_t dmaopt = dmaoptByTag(timer->tag);
    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaopt);
}

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured
#if defined(STM32F4) || defined(STM32F7)
                    && dma->channel == timer->dmaChannelConfigured
#endif
                    ) {
                    return j;
                }
            }

        }
    }

    return DMA_OPT_UNUSED;
}
#endif // USE_DMA_SPEC
