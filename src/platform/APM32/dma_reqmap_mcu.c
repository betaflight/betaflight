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

#include <stdint.h>

#include "platform.h"

#ifdef USE_DMA_SPEC

#include "timer_def.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "platform/adc_impl.h"
#include "pg/timerio.h"

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

#if defined(APM32F4)

#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _Stream ## s, DMA_CHANNEL_ ## c }

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    { DMA_PERIPH_SPI_SDO,  SPIDEV_1,  { DMA(2, 3, 3), DMA(2, 5, 3) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_1,  { DMA(2, 0, 3), DMA(2, 2, 3) } },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_2,  { DMA(1, 4, 0) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_2,  { DMA(1, 3, 0) } },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_3,  { DMA(1, 5, 0), DMA(1, 7, 0) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_3,  { DMA(1, 0, 0), DMA(1, 2, 0) } },
#endif // USE_SPI

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(2, 0, 0), DMA(2, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(2, 2, 1), DMA(2, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_3,  { DMA(2, 0, 2), DMA(2, 1, 2) } },
#endif // USE_ADC

#ifdef USE_SDCARD_SDIO
    { DMA_PERIPH_SDIO,    0,         { DMA(2, 3, 4), DMA(2, 6, 4) } },
#endif // USE_SDCARD_SDIO

#ifdef USE_UART1
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(2, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(2, 5, 4), DMA(2, 2, 4) } },
#endif
#ifdef USE_UART2
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(1, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(1, 5, 4) } },
#endif
#ifdef USE_UART3
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(1, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(1, 1, 4) } },
#endif
#ifdef USE_UART4
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(1, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(1, 2, 4) } },
#endif
#ifdef USE_UART5
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 0, 4) } },
#endif
#ifdef USE_UART6
    { DMA_PERIPH_UART_TX, UARTDEV_6, { DMA(2, 6, 5), DMA(2, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_6, { DMA(2, 1, 5), DMA(2, 2, 5) } },
#endif
};

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { TMR1, TC(CH1), { DMA(2, 6, 0), DMA(2, 1, 6), DMA(2, 3, 6) } },
    { TMR1, TC(CH2), { DMA(2, 6, 0), DMA(2, 2, 6) } },
    { TMR1, TC(CH3), { DMA(2, 6, 0), DMA(2, 6, 6) } },
    { TMR1, TC(CH4), { DMA(2, 4, 6) } },

    { TMR2, TC(CH1), { DMA(1, 5, 3) } },
    { TMR2, TC(CH2), { DMA(1, 6, 3) } },
    { TMR2, TC(CH3), { DMA(1, 1, 3) } },
    { TMR2, TC(CH4), { DMA(1, 7, 3), DMA(1, 6, 3) } },

    { TMR3, TC(CH1), { DMA(1, 4, 5) } },
    { TMR3, TC(CH2), { DMA(1, 5, 5) } },
    { TMR3, TC(CH3), { DMA(1, 7, 5) } },
    { TMR3, TC(CH4), { DMA(1, 2, 5) } },

    { TMR4, TC(CH1), { DMA(1, 0, 2) } },
    { TMR4, TC(CH2), { DMA(1, 3, 2) } },
    { TMR4, TC(CH3), { DMA(1, 7, 2) } },

    { TMR5, TC(CH1), { DMA(1, 2, 6) } },
    { TMR5, TC(CH2), { DMA(1, 4, 6) } },
    { TMR5, TC(CH3), { DMA(1, 0, 6) } },
    { TMR5, TC(CH4), { DMA(1, 1, 6), DMA(1, 3, 6) } },

    { TMR8, TC(CH1), { DMA(2, 2, 0), DMA(2, 2, 7) } },
    { TMR8, TC(CH2), { DMA(2, 2, 0), DMA(2, 3, 7) } },
    { TMR8, TC(CH3), { DMA(2, 2, 0), DMA(2, 4, 7) } },
    { TMR8, TC(CH4), { DMA(2, 7, 7) } },
};
#undef TC
#undef DMA
#endif // APM32F4

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

// dmaGetOptionByTimer is called by pgResetFn_timerIOConfig to find out dmaopt for pre-configured timer.

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured
#if defined(APM32F4)
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
