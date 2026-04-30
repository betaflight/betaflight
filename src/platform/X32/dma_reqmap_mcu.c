/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
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

#include "timer_def.h"
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/serial.h"
#include "drivers/adc.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#ifdef USE_TIMER_MGMT
#include "pg/timerio.h"
#endif

typedef struct dmaPeripheralMapping_s {
    dmaPeripheral_e device;
    uint8_t index;
    uint8_t dmaRequest;
} dmaPeripheralMapping_t;

typedef struct dmaTimerMapping_s {
    timerResource_t *tim;
    uint8_t channel;
    uint8_t dmaRequest;
} dmaTimerMapping_t;

#define REQMAP(periph, device) { DMA_PERIPH_ ## periph, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device }
#define REQMAP_DIR(periph, device, dir) { DMA_PERIPH_ ## periph ## _ ## dir, periph ## DEV_ ## device, DMA_REQUEST_ ## periph ## device ## _ ## dir }
#define REQMAP_TIMUP(timerNumber, request) { DMA_PERIPH_TIMUP, TIMER_INDEX(timerNumber), request }

#define DMA_REQUEST_UART1_RX DMAMUX1_REQUEST_USART1_RX
#define DMA_REQUEST_UART1_TX DMAMUX1_REQUEST_USART1_TX
#define DMA_REQUEST_UART2_RX DMAMUX1_REQUEST_USART2_RX
#define DMA_REQUEST_UART2_TX DMAMUX1_REQUEST_USART2_TX
#define DMA_REQUEST_UART3_RX DMAMUX1_REQUEST_USART3_RX
#define DMA_REQUEST_UART3_TX DMAMUX1_REQUEST_USART3_TX
#define DMA_REQUEST_UART4_RX DMAMUX1_REQUEST_USART4_RX
#define DMA_REQUEST_UART4_TX DMAMUX1_REQUEST_USART4_TX
#define DMA_REQUEST_UART5_RX DMAMUX1_REQUEST_USART5_RX
#define DMA_REQUEST_UART5_TX DMAMUX1_REQUEST_USART5_TX
#define DMA_REQUEST_UART6_RX DMAMUX1_REQUEST_USART6_RX
#define DMA_REQUEST_UART6_TX DMAMUX1_REQUEST_USART6_TX
#define DMA_REQUEST_UART7_RX DMAMUX1_REQUEST_USART7_RX
#define DMA_REQUEST_UART7_TX DMAMUX1_REQUEST_USART7_TX
#define DMA_REQUEST_UART8_RX DMAMUX1_REQUEST_USART8_RX
#define DMA_REQUEST_UART8_TX DMAMUX1_REQUEST_USART8_TX
#define DMA_REQUEST_UART9_RX DMAMUX1_REQUEST_UART9_RX
#define DMA_REQUEST_UART9_TX DMAMUX1_REQUEST_UART9_TX
#define DMA_REQUEST_UART10_RX DMAMUX1_REQUEST_UART10_RX
#define DMA_REQUEST_UART10_TX DMAMUX1_REQUEST_UART10_TX

#define DMA_REQUEST_SPI1_SDO DMAMUX1_REQUEST_SPI1_TX
#define DMA_REQUEST_SPI1_SDI DMAMUX1_REQUEST_SPI1_RX
#define DMA_REQUEST_SPI2_SDO DMAMUX1_REQUEST_SPI2_TX
#define DMA_REQUEST_SPI2_SDI DMAMUX1_REQUEST_SPI2_RX
#define DMA_REQUEST_SPI3_SDO DMAMUX1_REQUEST_SPI3_TX
#define DMA_REQUEST_SPI3_SDI DMAMUX1_REQUEST_SPI3_RX
#define DMA_REQUEST_SPI4_SDO DMAMUX1_REQUEST_SPI4_TX
#define DMA_REQUEST_SPI4_SDI DMAMUX1_REQUEST_SPI4_RX


#define DMA_REQUEST_ADC1     DMAMUX1_REQUEST_ADC1
#define DMA_REQUEST_ADC2     DMAMUX1_REQUEST_ADC2
#define DMA_REQUEST_ADC3     DMAMUX1_REQUEST_ADC3

static const dmaPeripheralMapping_t dmaPeripheralMapping[] = {
#ifdef USE_SPI_DEVICE_1
    REQMAP_DIR(SPI, 1, SDO),
    REQMAP_DIR(SPI, 1, SDI),
#endif
#ifdef USE_SPI_DEVICE_2
    REQMAP_DIR(SPI, 2, SDO),
    REQMAP_DIR(SPI, 2, SDI),
#endif
#ifdef USE_SPI_DEVICE_3
    REQMAP_DIR(SPI, 3, SDO),
    REQMAP_DIR(SPI, 3, SDI),
#endif
#ifdef USE_SPI_DEVICE_4
    REQMAP_DIR(SPI, 4, SDO),
    REQMAP_DIR(SPI, 4, SDI),
#endif

#ifdef USE_ADC
    REQMAP(ADC, 1),
    REQMAP(ADC, 2),
    REQMAP(ADC, 3),
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
#ifdef USE_UART8
    REQMAP_DIR(UART, 8, TX),
    REQMAP_DIR(UART, 8, RX),
#endif
#ifdef USE_UART9
    REQMAP_DIR(UART, 9, TX),
    REQMAP_DIR(UART, 9, RX),
#endif
#ifdef USE_UART10
    REQMAP_DIR(UART, 10, TX),
    REQMAP_DIR(UART, 10, RX),
#endif

#ifdef USE_TIMER
    REQMAP_TIMUP(X32_TIMER_ATIM1,  DMAMUX1_REQUEST_ATIM1_UP),
    REQMAP_TIMUP(X32_TIMER_ATIM2,  DMAMUX1_REQUEST_ATIM2_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA1, DMAMUX1_REQUEST_GTIMA1_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA2, DMAMUX1_REQUEST_GTIMA2_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA3, DMAMUX1_REQUEST_GTIMA3_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA4, DMAMUX1_REQUEST_GTIMA4_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA5, DMAMUX1_REQUEST_GTIMA5_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA6, DMAMUX1_REQUEST_GTIMA6_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMA7, DMAMUX1_REQUEST_GTIMA7_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMB1, DMAMUX1_REQUEST_GTIMB1_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMB2, DMAMUX1_REQUEST_GTIMB2_UP),
    REQMAP_TIMUP(X32_TIMER_GTIMB3, DMAMUX1_REQUEST_GTIMB3_UP),
    REQMAP_TIMUP(X32_TIMER_ATIM3,  DMAMUX1_REQUEST_ATIM3_UP),
    REQMAP_TIMUP(X32_TIMER_ATIM4,  DMAMUX1_REQUEST_ATIM4_UP),
#endif
};

#undef REQMAP_TIMUP
#undef REQMAP_DIR
#undef REQMAP

static const dmaTimerMapping_t dmaTimerMapping[] = {
    { (timerResource_t *)ATIM1,  TIM_CH_1, DMAMUX1_REQUEST_ATIM1_CH1 },
    { (timerResource_t *)ATIM1,  TIM_CH_2, DMAMUX1_REQUEST_ATIM1_CH2 },
    { (timerResource_t *)ATIM1,  TIM_CH_3, DMAMUX1_REQUEST_ATIM1_CH3 },
    { (timerResource_t *)ATIM1,  TIM_CH_4, DMAMUX1_REQUEST_ATIM1_CH4 },
    { (timerResource_t *)ATIM2,  TIM_CH_1, DMAMUX1_REQUEST_ATIM2_CH1 },
    { (timerResource_t *)ATIM2,  TIM_CH_2, DMAMUX1_REQUEST_ATIM2_CH2 },
    { (timerResource_t *)ATIM2,  TIM_CH_3, DMAMUX1_REQUEST_ATIM2_CH3 },
    { (timerResource_t *)ATIM2,  TIM_CH_4, DMAMUX1_REQUEST_ATIM2_CH4 },
    { (timerResource_t *)ATIM3,  TIM_CH_1, DMAMUX1_REQUEST_ATIM3_CH1 },
    { (timerResource_t *)ATIM3,  TIM_CH_2, DMAMUX1_REQUEST_ATIM3_CH2 },
    { (timerResource_t *)ATIM3,  TIM_CH_3, DMAMUX1_REQUEST_ATIM3_CH3 },
    { (timerResource_t *)ATIM3,  TIM_CH_4, DMAMUX1_REQUEST_ATIM3_CH4 },
    { (timerResource_t *)ATIM4,  TIM_CH_1, DMAMUX1_REQUEST_ATIM4_CH1 },
    { (timerResource_t *)ATIM4,  TIM_CH_2, DMAMUX1_REQUEST_ATIM4_CH2 },
    { (timerResource_t *)ATIM4,  TIM_CH_3, DMAMUX1_REQUEST_ATIM4_CH3 },
    { (timerResource_t *)ATIM4,  TIM_CH_4, DMAMUX1_REQUEST_ATIM4_CH4 },
    { (timerResource_t *)GTIMA1, TIM_CH_1, DMAMUX1_REQUEST_GTIMA1_CH1 },
    { (timerResource_t *)GTIMA1, TIM_CH_2, DMAMUX1_REQUEST_GTIMA1_CH2 },
    { (timerResource_t *)GTIMA1, TIM_CH_3, DMAMUX1_REQUEST_GTIMA1_CH3 },
    { (timerResource_t *)GTIMA1, TIM_CH_4, DMAMUX1_REQUEST_GTIMA1_CH4 },
    { (timerResource_t *)GTIMA2, TIM_CH_1, DMAMUX1_REQUEST_GTIMA2_CH1 },
    { (timerResource_t *)GTIMA2, TIM_CH_2, DMAMUX1_REQUEST_GTIMA2_CH2 },
    { (timerResource_t *)GTIMA2, TIM_CH_3, DMAMUX1_REQUEST_GTIMA2_CH3 },
    { (timerResource_t *)GTIMA2, TIM_CH_4, DMAMUX1_REQUEST_GTIMA2_CH4 },
    { (timerResource_t *)GTIMA3, TIM_CH_1, DMAMUX1_REQUEST_GTIMA3_CH1 },
    { (timerResource_t *)GTIMA3, TIM_CH_2, DMAMUX1_REQUEST_GTIMA3_CH2 },
    { (timerResource_t *)GTIMA3, TIM_CH_3, DMAMUX1_REQUEST_GTIMA3_CH3 },
    { (timerResource_t *)GTIMA3, TIM_CH_4, DMAMUX1_REQUEST_GTIMA3_CH4 },
    { (timerResource_t *)GTIMA4, TIM_CH_1, DMAMUX1_REQUEST_GTIMA4_CH1 },
    { (timerResource_t *)GTIMA4, TIM_CH_2, DMAMUX1_REQUEST_GTIMA4_CH2 },
    { (timerResource_t *)GTIMA4, TIM_CH_3, DMAMUX1_REQUEST_GTIMA4_CH3 },
    { (timerResource_t *)GTIMA4, TIM_CH_4, DMAMUX1_REQUEST_GTIMA4_CH4 },
    { (timerResource_t *)GTIMA5, TIM_CH_1, DMAMUX1_REQUEST_GTIMA5_CH1 },
    { (timerResource_t *)GTIMA5, TIM_CH_2, DMAMUX1_REQUEST_GTIMA5_CH2 },
    { (timerResource_t *)GTIMA5, TIM_CH_3, DMAMUX1_REQUEST_GTIMA5_CH3 },
    { (timerResource_t *)GTIMA5, TIM_CH_4, DMAMUX1_REQUEST_GTIMA5_CH4 },
    { (timerResource_t *)GTIMA6, TIM_CH_1, DMAMUX1_REQUEST_GTIMA6_CH1 },
    { (timerResource_t *)GTIMA6, TIM_CH_2, DMAMUX1_REQUEST_GTIMA6_CH2 },
    { (timerResource_t *)GTIMA6, TIM_CH_3, DMAMUX1_REQUEST_GTIMA6_CH3 },
    { (timerResource_t *)GTIMA6, TIM_CH_4, DMAMUX1_REQUEST_GTIMA6_CH4 },
    { (timerResource_t *)GTIMA7, TIM_CH_1, DMAMUX1_REQUEST_GTIMA7_CH1 },
    { (timerResource_t *)GTIMA7, TIM_CH_2, DMAMUX1_REQUEST_GTIMA7_CH2 },
    { (timerResource_t *)GTIMA7, TIM_CH_3, DMAMUX1_REQUEST_GTIMA7_CH3 },
    { (timerResource_t *)GTIMA7, TIM_CH_4, DMAMUX1_REQUEST_GTIMA7_CH4 },
    { (timerResource_t *)GTIMB1, TIM_CH_1, DMAMUX1_REQUEST_GTIMB1_CH1 },
    { (timerResource_t *)GTIMB1, TIM_CH_2, DMAMUX1_REQUEST_GTIMB1_CH2 },
    { (timerResource_t *)GTIMB1, TIM_CH_3, DMAMUX1_REQUEST_GTIMB1_CH3 },
    { (timerResource_t *)GTIMB1, TIM_CH_4, DMAMUX1_REQUEST_GTIMB1_CH4 },
    { (timerResource_t *)GTIMB2, TIM_CH_1, DMAMUX1_REQUEST_GTIMB2_CH1 },
    { (timerResource_t *)GTIMB2, TIM_CH_2, DMAMUX1_REQUEST_GTIMB2_CH2 },
    { (timerResource_t *)GTIMB2, TIM_CH_3, DMAMUX1_REQUEST_GTIMB2_CH3 },
    { (timerResource_t *)GTIMB2, TIM_CH_4, DMAMUX1_REQUEST_GTIMB2_CH4 },
    { (timerResource_t *)GTIMB3, TIM_CH_1, DMAMUX1_REQUEST_GTIMB3_CH1 },
    { (timerResource_t *)GTIMB3, TIM_CH_2, DMAMUX1_REQUEST_GTIMB3_CH2 },
    { (timerResource_t *)GTIMB3, TIM_CH_3, DMAMUX1_REQUEST_GTIMB3_CH3 },
    { (timerResource_t *)GTIMB3, TIM_CH_4, DMAMUX1_REQUEST_GTIMB3_CH4 },
};

#define DMA_CHANNEL_SPEC(dma, channel) { DMA_CODE(dma, channel, 0), (dmaResource_t *)DMA ## dma ## _CH ## channel, 0 }

static dmaChannelSpec_t dmaChannelSpec[] = {
    DMA_CHANNEL_SPEC(1, 0),
    DMA_CHANNEL_SPEC(1, 1),
    DMA_CHANNEL_SPEC(1, 2),
    DMA_CHANNEL_SPEC(1, 3),
    DMA_CHANNEL_SPEC(1, 4),
    DMA_CHANNEL_SPEC(1, 5),
    DMA_CHANNEL_SPEC(1, 6),
    DMA_CHANNEL_SPEC(1, 7),
    DMA_CHANNEL_SPEC(2, 0),
    DMA_CHANNEL_SPEC(2, 1),
    DMA_CHANNEL_SPEC(2, 2),
    DMA_CHANNEL_SPEC(2, 3),
    DMA_CHANNEL_SPEC(2, 4),
    DMA_CHANNEL_SPEC(2, 5),
    DMA_CHANNEL_SPEC(2, 6),
    DMA_CHANNEL_SPEC(2, 7),
    DMA_CHANNEL_SPEC(3, 0),
    DMA_CHANNEL_SPEC(3, 1),
    DMA_CHANNEL_SPEC(3, 2),
    DMA_CHANNEL_SPEC(3, 3),
    DMA_CHANNEL_SPEC(3, 4),
    DMA_CHANNEL_SPEC(3, 5),
    DMA_CHANNEL_SPEC(3, 6),
    DMA_CHANNEL_SPEC(3, 7),
};


#undef DMA_CHANNEL_SPEC

static void dmaSetupRequest(dmaChannelSpec_t *dmaSpec, uint8_t request)
{
    dmaSpec->dmaMuxId = request;
    const dmaCode_t code = dmaSpec->code;
    dmaSpec->code = DMA_CODE(DMA_CODE_CONTROLLER(code), DMA_CODE_STREAM(code), dmaSpec->dmaMuxId);
}

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    if (opt < 0 || opt >= (int8_t)ARRAYLEN(dmaChannelSpec)) {
        return NULL;
    }

    for (const dmaPeripheralMapping_t *periph = dmaPeripheralMapping; periph < ARRAYEND(dmaPeripheralMapping); periph++) {
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

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(timerResource_t *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    if (dmaopt < 0 || dmaopt >= (int8_t)ARRAYLEN(dmaChannelSpec)) {
        return NULL;
    }

    for (const dmaTimerMapping_t *timerMapping = dmaTimerMapping; timerMapping < ARRAYEND(dmaTimerMapping); timerMapping++) {
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

    return dmaGetChannelSpecByTimerValue(timer->tim, timer->channel, dmaoptByTag(timer->tag));
}

dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return DMA_OPT_UNUSED;
    }

    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaRefConfigured == dmaChannelSpec[opt].ref) {
            return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}

dmaoptValue_t dmaGetUpOptionByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return DMA_OPT_UNUSED;
    }

    for (unsigned opt = 0; opt < ARRAYLEN(dmaChannelSpec); opt++) {
        if (timer->dmaTimUPRef == dmaChannelSpec[opt].ref) {
            return (dmaoptValue_t)opt;
        }
    }

    return DMA_OPT_UNUSED;
}

#endif
