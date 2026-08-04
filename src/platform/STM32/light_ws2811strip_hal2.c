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

/*
 * WS2811 LED strip driver for HAL2 families (STM32C5).
 *
 * Uses LL TIM for PWM generation and direct LPDMA register writes
 * for DMA transfers, avoiding the HAL TIM/DMA layer.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/system.h"
#include "drivers/timer.h"
#include "platform/timer.h"

#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;
static dmaResource_t *ws2811DmaRef = NULL;
static TIM_TypeDef *ws2811Timer = NULL;
static uint16_t ws2811TimerChannel = 0;
static uint32_t ws2811DmaSource = 0;

static FAST_IRQ_HANDLER void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
        xLL_EX_DMA_DisableResource(ws2811DmaRef);
        CLEAR_BIT(ws2811Timer->DIER, ws2811DmaSource);
        ws2811LedDataTransferInProgress = false;
    }
}

bool ws2811LedStripHardwareInit(void)
{
    if (!ledStripIoTag) {
        return false;
    }

    const timerHardware_t *timerHardware = timerAllocate(ledStripIoTag, OWNER_LED_STRIP, 0);
    if (timerHardware == NULL) {
        return false;
    }

    ws2811Timer = (TIM_TypeDef *)timerHardware->tim;
    ws2811TimerChannel = timerHardware->channel;
    ws2811DmaSource = timerDmaSource(ws2811TimerChannel);

    dmaResource_t *dmaRef;
    uint32_t dmaChannel;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);
    if (dmaSpec == NULL) {
        return false;
    }
    dmaRef = dmaSpec->ref;
    dmaChannel = dmaSpec->channel;
#else
    dmaRef = timerHardware->dmaRef;
    dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }

    ws2811DmaRef = dmaRef;

    /* Timer base init */
    RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);
    LL_TIM_DisableCounter(ws2811Timer);

    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timerHardware->tim, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timerHardware->tim, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    LL_TIM_SetPrescaler(ws2811Timer, prescaler);
    LL_TIM_SetAutoReload(ws2811Timer, period);
    LL_TIM_SetCounterMode(ws2811Timer, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(ws2811Timer, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_GenerateEvent_UPDATE(ws2811Timer);

    /* GPIO */
    ws2811IO = IOGetByTag(ledStripIoTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN),
                   timerHardware->alternateFunction);

    /* OC config */
    uint32_t llChannel;
    switch (ws2811TimerChannel) {
    case TIM_CHANNEL_1: llChannel = LL_TIM_CHANNEL_CH1; break;
    case TIM_CHANNEL_2: llChannel = LL_TIM_CHANNEL_CH2; break;
    case TIM_CHANNEL_3: llChannel = LL_TIM_CHANNEL_CH3; break;
    case TIM_CHANNEL_4: llChannel = LL_TIM_CHANNEL_CH4; break;
    default: return false;
    }

    LL_TIM_OC_SetMode(ws2811Timer, llChannel, LL_TIM_OCMODE_PWM1);
    if (timerHardware->output & TIMER_OUTPUT_INVERTED) {
        LL_TIM_OC_SetPolarity(ws2811Timer, llChannel, LL_TIM_OCPOLARITY_LOW);
    } else {
        LL_TIM_OC_SetPolarity(ws2811Timer, llChannel, LL_TIM_OCPOLARITY_HIGH);
    }
    LL_TIM_OC_EnablePreload(ws2811Timer, llChannel);
    LL_TIM_CC_EnableChannel(ws2811Timer, llChannel);

    /* DMA setup using LL functions */
    DMA_Channel_TypeDef *dmaCh = (DMA_Channel_TypeDef *)dmaRef;

    dmaEnable(dmaGetIdentifier(dmaRef));

    xLL_EX_DMA_DeInit(dmaRef);

    LL_DMA_SetSrcIncMode(dmaCh, LL_DMA_SRC_ADDR_INCREMENTED);
    LL_DMA_SetDestIncMode(dmaCh, LL_DMA_DEST_ADDR_FIXED);
    LL_DMA_SetSrcDataWidth(dmaCh, LL_DMA_SRC_DATA_WIDTH_WORD);
    LL_DMA_SetDestDataWidth(dmaCh, LL_DMA_DEST_DATA_WIDTH_WORD);
    LL_DMA_SetPeriphRequest(dmaCh, dmaChannel);
    LL_DMA_SetSrcAddress(dmaCh, (uint32_t)ledStripDMABuffer);
    LL_DMA_SetDestAddress(dmaCh, (uint32_t)timerChCCR(timerHardware));
    LL_DMA_SetBlkDataLength(dmaCh, 0);

    xLL_EX_DMA_EnableIT_TC(dmaRef);

    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    LL_TIM_EnableAllOutputs(ws2811Timer);
    LL_TIM_EnableCounter(ws2811Timer);

    return true;
}

void ws2811LedStripStartTransfer(void)
{
    DMA_Channel_TypeDef *dmaCh = (DMA_Channel_TypeDef *)ws2811DmaRef;

    /* Disable channel before reconfiguring */
    xLL_EX_DMA_DisableResource(ws2811DmaRef);

    /* Update source address and transfer length */
    LL_DMA_SetSrcAddress(dmaCh, (uint32_t)ledStripDMABuffer);
    LL_DMA_SetBlkDataLength(dmaCh, WS2811_DMA_BUFFER_SIZE * sizeof(uint32_t));

    /* Reset timer counter */
    LL_TIM_SetCounter(ws2811Timer, 0);

    /* Enable DMA channel */
    xLL_EX_DMA_EnableResource(ws2811DmaRef);

    /* Enable timer DMA request for this channel */
    SET_BIT(ws2811Timer->DIER, ws2811DmaSource);
}
#endif
