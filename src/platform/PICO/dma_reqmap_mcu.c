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
#include "drivers/bus_spi.h"
#include "drivers/dma_reqmap.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"
#include "pg/timerio.h"

#define DMA(c) { (c), (dmaResource_t *) dma_hw->ch[c] , 0 }

static dmaChannelSpec_t dmaChannelSpec[MAX_PERIPHERAL_DMA_OPTIONS] = {
    DMA(1),
    DMA(2),
    DMA(3),
    DMA(4),
    DMA(5),
    DMA(6),
    DMA(7),
    DMA(8),
    DMA(9),
    DMA(10),
    DMA(11),
    DMA(12),
#ifdef RP2350
    DMA(13),
    DMA(14),
    DMA(15),
    DMA(16),
#endif
};

#undef DMA

const dmaChannelSpec_t *dmaGetChannelSpecByPeripheral(dmaPeripheral_e device, uint8_t index, int8_t opt)
{
    UNUSED(device);
    UNUSED(index);
    UNUSED(opt);
    //TODO : Implementation for PICO
    return NULL;
}

dmaoptValue_t dmaoptByTag(ioTag_t ioTag)
{
    UNUSED(ioTag);
    //TODO : Implementation for PICO
    return DMA_OPT_UNUSED;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimerValue(TIM_TypeDef *tim, uint8_t channel, dmaoptValue_t dmaopt)
{
    //TODO : Implementation for PICO
    return NULL;
}

const dmaChannelSpec_t *dmaGetChannelSpecByTimer(const timerHardware_t *timer)
{
    if (!timer) {
        return NULL;
    }

    //TODO : Implementation for PICO
    return NULL;
}

// dmaGetOptionByTimer is called by pgResetFn_timerIOConfig to find out dmaopt for pre-configured timer.
dmaoptValue_t dmaGetOptionByTimer(const timerHardware_t *timer)
{
    //TODO : Implementation for PICO
    return DMA_OPT_UNUSED;
}

// A variant of dmaGetOptionByTimer that looks for matching dmaTimUPRef
dmaoptValue_t dmaGetUpOptionByTimer(const timerHardware_t *timer)
{
    //TODO : Implementation for PICO
    return DMA_OPT_UNUSED;
}

#endif // USE_DMA_SPEC
