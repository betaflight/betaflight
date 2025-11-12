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
#include "drivers/adc.h"
#include "platform/adc_impl.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "pg/timerio.h"

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    uint8_t dmaRequest;
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    TIM_TypeDef *tim;
    uint8_t channel;
    uint8_t dmaRequest;
} dmaTimerMapping_t;

#define REQMAP_SGL(periph) { DMA_PERIPH_ ## periph, 0, DMA_REQUEST_ ## periph }
#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP_TIMUP(periph, timno) { DMA_PERIPH_TIMUP, TIMER_INDEX(timno), DMAMUX_DMAREQ_ID_ ## TMR ## timno ## _OVERFLOW }

#define DMA_REQUEST_UART1_RX DMAMUX_DMAREQ_ID_USART1_RX
#define DMA_REQUEST_UART1_TX DMAMUX_DMAREQ_ID_USART1_TX
#define DMA_REQUEST_UART2_RX DMAMUX_DMAREQ_ID_USART2_RX
#define DMA_REQUEST_UART2_TX DMAMUX_DMAREQ_ID_USART2_TX
#define DMA_REQUEST_UART3_RX DMAMUX_DMAREQ_ID_USART3_RX
#define DMA_REQUEST_UART3_TX DMAMUX_DMAREQ_ID_USART3_TX
#define DMA_REQUEST_UART4_RX DMAMUX_DMAREQ_ID_UART4_RX
#define DMA_REQUEST_UART4_TX DMAMUX_DMAREQ_ID_UART4_TX
#define DMA_REQUEST_UART5_RX DMAMUX_DMAREQ_ID_UART5_RX
#define DMA_REQUEST_UART5_TX DMAMUX_DMAREQ_ID_UART5_TX

#define DMA_REQUEST_UART6_RX DMAMUX_DMAREQ_ID_USART6_RX
#define DMA_REQUEST_UART6_TX DMAMUX_DMAREQ_ID_USART6_TX

// Resolve our preference for SDO/SDI rather than TX/RX
#define DMA_REQUEST_SPI1_SDO DMAMUX_DMAREQ_ID_SPI1_TX
#define DMA_REQUEST_SPI1_SDI DMAMUX_DMAREQ_ID_SPI1_RX
#define DMA_REQUEST_SPI2_SDO DMAMUX_DMAREQ_ID_SPI2_TX
#define DMA_REQUEST_SPI2_SDI DMAMUX_DMAREQ_ID_SPI2_RX
#define DMA_REQUEST_SPI3_SDO DMAMUX_DMAREQ_ID_SPI3_TX
#define DMA_REQUEST_SPI3_SDI DMAMUX_DMAREQ_ID_SPI3_RX
#define DMA_REQUEST_SPI4_SDO DMAMUX_DMAREQ_ID_SPI4_TX
#define DMA_REQUEST_SPI4_SDI DMAMUX_DMAREQ_ID_SPI4_RX

#define DMA_REQUEST_ADC1 DMAMUX_DMAREQ_ID_ADC1
#define DMA_REQUEST_ADC2 DMAMUX_DMAREQ_ID_ADC2
#define DMA_REQUEST_ADC3 DMAMUX_DMAREQ_ID_ADC3

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI
    REQMAP_DIR(SPI, 1, SDO),
    REQMAP_DIR(SPI, 1, SDI),
    REQMAP_DIR(SPI, 2, SDO),
    REQMAP_DIR(SPI, 2, SDI),
    REQMAP_DIR(SPI, 3, SDO),
    REQMAP_DIR(SPI, 3, SDI),
    REQMAP_DIR(SPI, 4, SDO),
    REQMAP_DIR(SPI, 4, SDI),
#endif // USE_SPI

#ifdef USE_ADC
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
    REQMAP(ADC, 3),
#endif

    // UARTDEV_x enum value exists only when coresponding UART is enabled
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

#ifdef USE_TIMER
    REQMAP_TIMUP(TIMUP, 1),
    REQMAP_TIMUP(TIMUP, 2),
    REQMAP_TIMUP(TIMUP, 3),
    REQMAP_TIMUP(TIMUP, 4),
    REQMAP_TIMUP(TIMUP, 5),
    REQMAP_TIMUP(TIMUP, 6),
    REQMAP_TIMUP(TIMUP, 7),
    REQMAP_TIMUP(TIMUP, 8),
    REQMAP_TIMUP(TIMUP, 20),
#endif
};

#undef REQMAP_TIMUP
#undef REQMAP
#undef REQMAP_SGL
#undef REQMAP_DIR

#define TC(chan) DEF_TIM_CHANNEL(CH_ ## chan)

#define REQMAP_TIM(tim, chan) { tim, TC(chan), DMAMUX_DMAREQ_ID_ ## tim ## _ ## chan }

static const dmaTimerMapping_t dmaTimerMapping[] = {
    REQMAP_TIM(TMR1, CH1),
    REQMAP_TIM(TMR1, CH2),
    REQMAP_TIM(TMR1, CH3),
    REQMAP_TIM(TMR1, CH4),
    REQMAP_TIM(TMR2, CH1),
    REQMAP_TIM(TMR2, CH2),
    REQMAP_TIM(TMR2, CH3),
    REQMAP_TIM(TMR2, CH4),
    REQMAP_TIM(TMR3, CH1),
    REQMAP_TIM(TMR3, CH2),
    REQMAP_TIM(TMR3, CH3),
    REQMAP_TIM(TMR3, CH4),
    REQMAP_TIM(TMR4, CH1),
    REQMAP_TIM(TMR4, CH2),
    REQMAP_TIM(TMR4, CH3),
    REQMAP_TIM(TMR4, CH4),
    REQMAP_TIM(TMR5, CH1),
    REQMAP_TIM(TMR5, CH2),
    REQMAP_TIM(TMR5, CH3),
    REQMAP_TIM(TMR5, CH4),
    REQMAP_TIM(TMR8, CH1),
    REQMAP_TIM(TMR8, CH2),
    REQMAP_TIM(TMR8, CH3),
    REQMAP_TIM(TMR8, CH4),
    REQMAP_TIM(TMR20, CH1),
    REQMAP_TIM(TMR20, CH2),
    REQMAP_TIM(TMR20, CH3),
    REQMAP_TIM(TMR20, CH4),
    // XXX Check non-CH1 for TIM15,16,17 and 20
};

#undef TC
#undef REQMAP_TIM

#define DMA(d, c) { DMA_CODE(d, c, 0), (dmaResource_t *) DMA ## d ## _CHANNEL ## c , 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1, 1),
    DMA(1, 2),
    DMA(1, 3),
    DMA(1, 4),
    DMA(1, 5),
    DMA(1, 6),
    DMA(1, 7),
    DMA(2, 1),
    DMA(2, 2),
    DMA(2, 3),
    DMA(2, 4),
    DMA(2, 5),
    DMA(2, 6),
    DMA(2, 7),
};

#undef DMA

static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    dmaSpec->dmaMuxId = request;
    dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->dmaMuxId);
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaPeripheralMapping) ; i++) {
        const dmaPeripheralMapping_t *periph = &dmaPeripheralMapping[i];
        if (periph->device == device && periph->index == index) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[opt];
            dmaSetupRequest(dmaSpec, periph->dmaRequest);
            return dmaSpec;
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
    if (dmaopt < 0 || dmaopt >= MAX_PERIPHERAL_DMA_OPTIONS) {
        return NULL;
    }

    for (unsigned i = 0 ; i < ARRAYLEN(dmaTimerMapping) ; i++) {
        const dmaTimerMapping_t *timerMapping = &dmaTimerMapping[i];
        if (timerMapping->tim == tim && timerMapping->channel == channel) {
            dmaChannelSpec_t *dmaSpec = &dmaChannelSpec[dmaopt];
            dmaSetupRequest(dmaSpec, timerMapping->dmaRequest);
            return dmaSpec;
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
    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaRefConfigured == dmaChannelSpec[opt].ref) {
                return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}

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

#endif // USE_DMA_SPEC
