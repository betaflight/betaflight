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
#include "platform/adc_impl.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "pg/timerio.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
#if defined(GD32H7)
    uint8_t dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_PERIPHERAL_DMA_OPTIONS];
#endif
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    timerResource_t *tim;
    uint8_t channel;
#if defined(GD32H7)
    uint8_t dmaRequest;
#else
    dmaChannelSpec_t channelSpec[MAX_TIMER_DMA_OPTIONS];
#endif
} dmaTimerMapping_t;

#if defined(GD32F4)
#define DMA(d, s, c) { DMA_CODE(d, s, c), (dmaResource_t *)DMA ## d ## _CH ## s ## _BASE, DMA_SUBPERI ## c }

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    { DMA_PERIPH_SPI_SDO,  SPIDEV_0,  { DMA(1, 3, 3), DMA(1, 5, 3) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_0,  { DMA(1, 0, 3), DMA(1, 2, 3) } },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_1,  { DMA(0, 4, 0) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_1,  { DMA(0, 3, 0) } },
    { DMA_PERIPH_SPI_SDO,  SPIDEV_2,  { DMA(0, 5, 0), DMA(0, 7, 0) } },
    { DMA_PERIPH_SPI_SDI,  SPIDEV_2,  { DMA(0, 0, 0), DMA(0, 2, 0) } },

#endif

#ifdef USE_ADC
    { DMA_PERIPH_ADC,     ADCDEV_0,  { DMA(1, 0, 0), DMA(1, 4, 0) } },
    { DMA_PERIPH_ADC,     ADCDEV_1,  { DMA(1, 2, 1), DMA(1, 3, 1) } },
    { DMA_PERIPH_ADC,     ADCDEV_2,  { DMA(1, 0, 2), DMA(1, 1, 2) } },
#endif

#ifdef USE_SDCARD_SDIO
    { DMA_PERIPH_SDIO,    0,         { DMA(1, 3, 4), DMA(1, 6, 4) } },
#endif

#ifdef USE_UART0
    { DMA_PERIPH_UART_TX, UARTDEV_0, { DMA(1, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_0, { DMA(1, 5, 4), DMA(1, 2, 4) } },
#endif

#ifdef USE_UART1
    { DMA_PERIPH_UART_TX, UARTDEV_1, { DMA(0, 6, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_1, { DMA(0, 5, 4) } },
#endif

#ifdef USE_UART2
    { DMA_PERIPH_UART_TX, UARTDEV_2, { DMA(0, 3, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_2, { DMA(0, 1, 4) } },
#endif

#ifdef USE_UART3
    { DMA_PERIPH_UART_TX, UARTDEV_3, { DMA(0, 4, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_3, { DMA(0, 2, 4) } },
#endif

#ifdef USE_UART4
    { DMA_PERIPH_UART_TX, UARTDEV_4, { DMA(0, 7, 4) } },
    { DMA_PERIPH_UART_RX, UARTDEV_4, { DMA(0, 0, 4) } },
#endif

#ifdef USE_UART5
    { DMA_PERIPH_UART_TX, UARTDEV_5, { DMA(1, 6, 5), DMA(1, 7, 5) } },
    { DMA_PERIPH_UART_RX, UARTDEV_5, { DMA(1, 1, 5), DMA(1, 2, 5) } },
#endif
};

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

static const dmaTimerMapping_t dmaTimerMapping[] = {
    // Generated from 'timer_def.h'
    { (timerResource_t *)TIMER0, TC(CH0), { DMA(1, 6, 0), DMA(1, 1, 6), DMA(1, 3, 6) } },
    { (timerResource_t *)TIMER0, TC(CH1), { DMA(1, 6, 0), DMA(1, 2, 6) } },
    { (timerResource_t *)TIMER0, TC(CH2), { DMA(1, 6, 0), DMA(1, 6, 6) } },
    { (timerResource_t *)TIMER0, TC(CH3), { DMA(1, 4, 6) } },

    { (timerResource_t *)TIMER1, TC(CH0), { DMA(0, 5, 3) } },
    { (timerResource_t *)TIMER1, TC(CH1), { DMA(0, 6, 3) } },
    { (timerResource_t *)TIMER1, TC(CH2), { DMA(0, 1, 3) } },
    { (timerResource_t *)TIMER1, TC(CH3), { DMA(0, 7, 3), DMA(0, 6, 3) } },

    { (timerResource_t *)TIMER2, TC(CH0), { DMA(0, 4, 5) } },
    { (timerResource_t *)TIMER2, TC(CH1), { DMA(0, 5, 5) } },
    { (timerResource_t *)TIMER2, TC(CH2), { DMA(0, 7, 5) } },
    { (timerResource_t *)TIMER2, TC(CH3), { DMA(0, 2, 5) } },

    { (timerResource_t *)TIMER3, TC(CH0), { DMA(0, 0, 2) } },
    { (timerResource_t *)TIMER3, TC(CH1), { DMA(0, 3, 2) } },
    { (timerResource_t *)TIMER3, TC(CH2), { DMA(0, 7, 2) } },

    { (timerResource_t *)TIMER4, TC(CH0), { DMA(0, 2, 6) } },
    { (timerResource_t *)TIMER4, TC(CH1), { DMA(0, 4, 6) } },
    { (timerResource_t *)TIMER4, TC(CH2), { DMA(0, 0, 6) } },
    { (timerResource_t *)TIMER4, TC(CH3), { DMA(0, 1, 6), DMA(0, 3, 6) } },

    { (timerResource_t *)TIMER7, TC(CH0), { DMA(1, 2, 0), DMA(1, 2, 7) } },
    { (timerResource_t *)TIMER7, TC(CH1), { DMA(1, 2, 0), DMA(1, 3, 7) } },
    { (timerResource_t *)TIMER7, TC(CH2), { DMA(1, 2, 0), DMA(1, 4, 7) } },
    { (timerResource_t *)TIMER7, TC(CH3), { DMA(1, 7, 7) } },
};

#undef TC
#undef DMA
#elif defined(GD32H7)

#define REQMAP_SGL(periph) { DMA_PERIPH_ ## periph, 0, DMA_REQUEST_ ## periph }
#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP_TIMUP(periph, timno) { DMA_PERIPH_TIMUP, timno, DMA_REQUEST_ ## periph ## timno ## _UP }

// Resolve UART/USART mess
#define DMA_REQUEST_UART0_RX DMA_REQUEST_USART0_RX
#define DMA_REQUEST_UART0_TX DMA_REQUEST_USART0_TX
#define DMA_REQUEST_UART1_RX DMA_REQUEST_USART1_RX
#define DMA_REQUEST_UART1_TX DMA_REQUEST_USART1_TX
#define DMA_REQUEST_UART2_RX DMA_REQUEST_USART2_RX
#define DMA_REQUEST_UART2_TX DMA_REQUEST_USART2_TX
#define DMA_REQUEST_UART5_RX DMA_REQUEST_USART5_RX
#define DMA_REQUEST_UART5_TX DMA_REQUEST_USART5_TX

// Resolve our preference for SDO/SDI rather than TX/RX
#define DMA_REQUEST_SPI0_SDO DMA_REQUEST_SPI0_TX
#define DMA_REQUEST_SPI0_SDI DMA_REQUEST_SPI0_RX
#define DMA_REQUEST_SPI1_SDO DMA_REQUEST_SPI1_TX
#define DMA_REQUEST_SPI1_SDI DMA_REQUEST_SPI1_RX
#define DMA_REQUEST_SPI2_SDO DMA_REQUEST_SPI2_TX
#define DMA_REQUEST_SPI2_SDI DMA_REQUEST_SPI2_RX
#define DMA_REQUEST_SPI3_SDO DMA_REQUEST_SPI3_TX
#define DMA_REQUEST_SPI3_SDI DMA_REQUEST_SPI3_RX
#define DMA_REQUEST_SPI4_SDO DMA_REQUEST_SPI4_TX
#define DMA_REQUEST_SPI4_SDI DMA_REQUEST_SPI4_RX
#define DMA_REQUEST_SPI5_SDO DMA_REQUEST_SPI5_TX
#define DMA_REQUEST_SPI5_SDI DMA_REQUEST_SPI5_RX

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    REQMAP_DIR(SPI, 0, SDO),
    REQMAP_DIR(SPI, 0, SDI),
    REQMAP_DIR(SPI, 1, SDO),
    REQMAP_DIR(SPI, 1, SDI),
    REQMAP_DIR(SPI, 2, SDO),
    REQMAP_DIR(SPI, 2, SDI),
    REQMAP_DIR(SPI, 3, SDO),
    REQMAP_DIR(SPI, 3, SDI),
    // REQMAP_DIR(SPI, 4, SDO),
    // REQMAP_DIR(SPI, 4, SDI),
    // REQMAP_DIR(SPI, 5, SDO),
    // REQMAP_DIR(SPI, 5, SDI),

#endif // USE_SPI

#ifdef USE_ADC
    REQMAP(ADC, 0),
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
#endif

#ifdef USE_UART0
    REQMAP_DIR(UART, 0, TX),
    REQMAP_DIR(UART, 0, RX),
#endif
#ifdef USE_UART1
    REQMAP_DIR(UART, 1, TX),
    REQMAP_DIR(UART, 1, RX),
#endif
#ifdef USE_UART2
    REQMAP_DIR(UART, 2, TX),
    REQMAP_DIR(UART, 2, RX),
#endif
#ifdef USE_UART3
    REQMAP_DIR(UART, 3, TX),
    REQMAP_DIR(UART, 3, RX),
#endif
#ifdef USE_UART4
    REQMAP_DIR(UART, 4, TX),
    REQMAP_DIR(UART, 4, RX),
#endif
#ifdef USE_UART5
    REQMAP_DIR(UART, 5, TX),
    REQMAP_DIR(UART, 5, RX),
#endif
#ifdef USE_UART6
    REQMAP_DIR(UART, 6, TX),
    REQMAP_DIR(UART, 6, RX),
#endif
#ifdef USE_UART7
    REQMAP_DIR(UART, 7, TX),
    REQMAP_DIR(UART, 7, RX),
#endif


#ifdef USE_TIMER
// Pseudo peripheral for TIMx_UP channel
    REQMAP_TIMUP(TIMER, 0),
    REQMAP_TIMUP(TIMER, 1),
    REQMAP_TIMUP(TIMER, 2),
    REQMAP_TIMUP(TIMER, 3),
    REQMAP_TIMUP(TIMER, 4),
    REQMAP_TIMUP(TIMER, 5),
    REQMAP_TIMUP(TIMER, 6),
    REQMAP_TIMUP(TIMER, 7),
    REQMAP_TIMUP(TIMER, 14),
    REQMAP_TIMUP(TIMER, 15),
    REQMAP_TIMUP(TIMER, 16),
    REQMAP_TIMUP(TIMER, 22),
    REQMAP_TIMUP(TIMER, 23),
    REQMAP_TIMUP(TIMER, 30),
    REQMAP_TIMUP(TIMER, 31),
    REQMAP_TIMUP(TIMER, 40),
    REQMAP_TIMUP(TIMER, 41),
    REQMAP_TIMUP(TIMER, 42),
    REQMAP_TIMUP(TIMER, 43),
    REQMAP_TIMUP(TIMER, 44),
    REQMAP_TIMUP(TIMER, 50),
    REQMAP_TIMUP(TIMER, 51),

#endif
};

#undef REQMAP_TIMUP
#undef REQMAP
#undef REQMAP_SGL
#undef REQMAP_DIR

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

#define REQMAP_TIM(tim, chan) { (timerResource_t *)tim, TC(chan), DMA_REQUEST_ ## tim ## _ ## chan }

static const dmaTimerMapping_t dmaTimerMapping[] = {
    REQMAP_TIM(TIMER0, CH0),
    REQMAP_TIM(TIMER0, CH1),
    REQMAP_TIM(TIMER0, CH2),
    REQMAP_TIM(TIMER0, CH3),
    REQMAP_TIM(TIMER1, CH0),
    REQMAP_TIM(TIMER1, CH1),
    REQMAP_TIM(TIMER1, CH2),
    REQMAP_TIM(TIMER1, CH3),
    REQMAP_TIM(TIMER2, CH0),
    REQMAP_TIM(TIMER2, CH1),
    REQMAP_TIM(TIMER2, CH2),
    REQMAP_TIM(TIMER2, CH3),
    REQMAP_TIM(TIMER3, CH0),
    REQMAP_TIM(TIMER3, CH1),
    REQMAP_TIM(TIMER3, CH2),
    REQMAP_TIM(TIMER3, CH3),
    REQMAP_TIM(TIMER4, CH0),
    REQMAP_TIM(TIMER4, CH1),
    REQMAP_TIM(TIMER4, CH2),
    REQMAP_TIM(TIMER4, CH3),
    REQMAP_TIM(TIMER7, CH0),
    REQMAP_TIM(TIMER7, CH1),
    REQMAP_TIM(TIMER7, CH2),
    REQMAP_TIM(TIMER7, CH3),
    REQMAP_TIM(TIMER14, CH0),
    REQMAP_TIM(TIMER14, CH1),
    REQMAP_TIM(TIMER15, CH0),
    REQMAP_TIM(TIMER16, CH0),
    REQMAP_TIM(TIMER22, CH0),
    REQMAP_TIM(TIMER22, CH1),
    REQMAP_TIM(TIMER22, CH2),
    REQMAP_TIM(TIMER22, CH3),
    REQMAP_TIM(TIMER23, CH0),
    REQMAP_TIM(TIMER23, CH1),
    REQMAP_TIM(TIMER23, CH2),
    REQMAP_TIM(TIMER23, CH3),
    REQMAP_TIM(TIMER30, CH0),
    REQMAP_TIM(TIMER30, CH1),
    REQMAP_TIM(TIMER30, CH2),
    REQMAP_TIM(TIMER30, CH3),
    REQMAP_TIM(TIMER31, CH0),
    REQMAP_TIM(TIMER31, CH1),
    REQMAP_TIM(TIMER31, CH2),
    REQMAP_TIM(TIMER31, CH3),
    REQMAP_TIM(TIMER40, CH0),
    REQMAP_TIM(TIMER40, CH1),
    REQMAP_TIM(TIMER41, CH0),
    REQMAP_TIM(TIMER41, CH1),
    REQMAP_TIM(TIMER42, CH0),
    REQMAP_TIM(TIMER42, CH1),
    REQMAP_TIM(TIMER43, CH0),
    REQMAP_TIM(TIMER43, CH1),
    REQMAP_TIM(TIMER44, CH0),
    REQMAP_TIM(TIMER44, CH1),
};

#undef TC
#undef REQMAP_TIM

#define DMA(d, s) { DMA_CODE(d, s, 0), (dmaResource_t *)DMA ## d ## _CH ## s ## _BASE, 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(0, 0),
    DMA(0, 1),
    DMA(0, 2),
    DMA(0, 3),
    DMA(0, 4),
    DMA(0, 5),
    DMA(0, 6),
    DMA(0, 7),
    DMA(1, 0),
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
};

#undef DMA

#endif

#if defined(GD32H7)
static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    // Setup request as channel
    dmaSpec->channel = request;

    // Insert DMA request into code
    dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->channel);
}
#endif

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (const dmaPeripheralMapping_t *periph =  dmaPeripheralMapping; periph < ARRAYEND(dmaPeripheralMapping); periph++) {
#if defined(GD32H7)
        if (periph->device == device && periph->index == index) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[opt];
            dmaSetupRequest(dmaSpec, periph->dmaRequest);
            return dmaSpec;
        }
#else
        if (periph->device == device && periph->index == index && periph->channelSpec[opt].ref) {
            return &periph->channelSpec[opt];
        }
#endif
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

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(timerResource_t *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= MAX_TIMER_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
#if defined(GD32H7)
        if (timerMapping->tim == tim && timerMapping->channel == channel) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[dmaopt];
            dmaSetupRequest(dmaSpec, timerMapping->dmaRequest);
            return dmaSpec;
        }
#else
        if (timerMapping->tim == tim && timerMapping->channel == channel && timerMapping->channelSpec[dmaopt].ref) {
            return &timerMapping->channelSpec[dmaopt];
        }
#endif
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
#if defined(GD32H7)
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaRefConfigured == dmaChannelSpec[opt].ref) {
                return (dmaoptValue_t)opt;
        }
    }
#else
    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping); i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == timer->tim && timerMapping->channel == timer->channel) {
            for (unsigned j = 0; j < MAX_TIMER_DMA_OPTIONS; j++) {
                const dmaChannelSpec_t *dma = &timerMapping->channelSpec[j];
                if (dma->ref == timer->dmaRefConfigured && dma->channel == timer->dmaChannelConfigured) {
                    return j;
                }
            }
        }
    }
#endif

    return DMA_OPT_UNUSED;
}

#if defined(GD32H7)
// A variant of dmaGetOptionByTimer that looks for matching dmaTimUPRef
dmaoptValue_t dmaGetUpOptionByTimer(const timerHardware_t *timer)
{
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaTimUPRef == dmaChannelSpec[opt].ref) {
                return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}
#endif

#endif // USE_DMA_SPEC
