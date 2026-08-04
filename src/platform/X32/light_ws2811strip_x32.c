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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "platform/dma.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"
#include "platform/timer.h"

#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;

static dmaResource_t *dmaRef = NULL;
static TIM_TypeDef *timer = NULL;
static uint16_t timerDmaRequest = 0;

static void ws2811CleanDmaBuffer(void)
{
#ifdef USE_LED_STRIP_CACHE_MGMT
    X32_CLEAN_DCACHE_BY_ADDR(ledStripDMABuffer, WS2811_DMA_BUF_CACHE_ALIGN_BYTES);
#endif
}

static void ws2811DisableTransfer(dmaResource_t *ref)
{
    xDMA_Cmd(ref, DISABLE);
    if (timerDmaRequest) {
        TIM_EnableDma(timer, timerDmaRequest, DISABLE);
    }
}

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
    const uint32_t status = DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);

    if (status & (DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF)) {
        ws2811DisableTransfer(descriptor->ref);
        ws2811LedDataTransferInProgress = false;
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
        return;
    }

    if (!(status & DMA_IT_TCIF)) {
        return;
    }

#if defined(USE_WS2811_SINGLE_COLOUR)
    static uint32_t counter = 0;

    counter++;
    if (counter == WS2811_LED_STRIP_LENGTH) {
        memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
        ws2811CleanDmaBuffer();
    } else if (counter == (WS2811_LED_STRIP_LENGTH + WS2811_DELAY_ITERATIONS)) {
        counter = 0;
        ws2811DisableTransfer(descriptor->ref);
        ws2811LedDataTransferInProgress = false;
    }
#else
    ws2811DisableTransfer(descriptor->ref);
    ws2811LedDataTransferInProgress = false;
#endif

    DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
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

    timer = (TIM_TypeDef *)timerHardware->tim;

    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }

    ws2811IO = IOGetByTag(ledStripIoTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    RCC_ClockCmd(timerRCC(timerHardware->tim), ENABLE);
    TIM_Enable(timer, DISABLE);

    const uint16_t prescaler = timerGetPrescalerByDesiredMhz(timerHardware->tim, WS2811_TIMER_MHZ);
    const uint16_t period = timerGetPeriodByPrescaler(timerHardware->tim, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    TIM_TimeBaseInitTypeDef timeBaseInit;
    TIM_InitTimBaseStruct(&timeBaseInit);
    timeBaseInit.Period = (period > 0) ? period - 1U : 0U;
    timeBaseInit.Prescaler = prescaler;
    timeBaseInit.CounterMode = TIM_CNT_MODE_UP;
    timeBaseInit.ClkDiv = TIM_CLK_DIV1;
    timeBaseInit.RepetCnt = 0;
    TIM_InitTimeBase(timer, &timeBaseInit);
    TIM_ConfigArPreload(timer, ENABLE);

    TIM_OCInitTypeDef ocInit;
    TIM_OCStructInit(&ocInit);
    ocInit.OCMode = TIM_OCMODE_PWM1;
    ocInit.Pulse = 0;

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        ocInit.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
        ocInit.OCNIdleState = TIM_OCN_IDLE_STATE_RESET;
        ocInit.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OCN_POLARITY_LOW : TIM_OCN_POLARITY_HIGH;
    } else {
        ocInit.OutputState = TIM_OUTPUT_STATE_ENABLE;
        ocInit.OCIdleState = TIM_OC_IDLE_STATE_SET;
        ocInit.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIM_OC_POLARITY_LOW : TIM_OC_POLARITY_HIGH;
    }

    timerOCInit(timer, timerHardware->channel, &ocInit);
    timerOCPreloadConfig(timer, timerHardware->channel, TIM_OC_PRE_LOAD_ENABLE);

    timerDmaRequest = timerDmaSource(timerHardware->channel);
    TIM_EnableDma(timer, timerDmaRequest, DISABLE);

    const dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);
    dmaEnable(dmaIdentifier);
    dmaMuxEnable(dmaIdentifier, dmaSpec->dmaMuxId);
    dmaSetHandler(dmaIdentifier, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    DMA_InitTypeDef dmaInit;
    DMA_ChannelStructInit(&dmaInit);
    dmaInit.IntEn = 0x1U;
    dmaInit.DstAddr = (uint32_t)timerCCR(timerHardware->tim, timerHardware->channel);
    dmaInit.SrcAddr = (uint32_t)ledStripDMABuffer;
    dmaInit.SrcTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
    dmaInit.DstTfrWidth = DMA_CH_TRANSFER_WIDTH_32;
    dmaInit.SrcAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_INCREMENT;
    dmaInit.DstAddrCountMode = DMA_CH_ADDRESS_COUNT_MODE_NO_CHANGE;
    dmaInit.SrcBurstLen = DMA_CH_BURST_LENGTH_1;
    dmaInit.DstBurstLen = DMA_CH_BURST_LENGTH_1;
    dmaInit.TfrTypeFlowCtrl = DMA_CH_TRANSFER_FLOW_M2P_DMA;
    dmaInit.BlkTfrSize = WS2811_DMA_BUFFER_SIZE;
    dmaInit.TfrType = DMA_CH_TRANSFER_TYPE_SINGLE_BLOCK;
    dmaInit.ChannelPriority = DMA_CH_PRIORITY_3;
    dmaInit.SrcHandshaking = DMA_CH_SRC_HANDSHAKING_SOFTWARE;
    dmaInit.DstHandshaking = DMA_CH_DST_HANDSHAKING_HARDWARE;
    dmaInit.DstHsInterface = dmaX32HandshakeInterfaceFromResource((DMA_ARCH_TYPE *)dmaRef);
    dmaInit.DstHsInterfacePol = DMA_CH_HANDSHAKING_IF_POL_H;

#if defined(USE_WS2811_SINGLE_COLOUR)
    dmaInit.TfrType = DMA_CH_TRANSFER_TYPE_MULTI_BLOCK_SRCADR_RELOAD_DSTADR_CONTIGUOUS;
#endif

    xDMA_Init(dmaRef, &dmaInit);
    xDMA_ITConfig(dmaRef, DMA_IT_TCIF | DMA_IT_TEIF, ENABLE);

    timerChannelEnable(timerHardware);
    timerStart(timerHardware);

    return true;
}

void ws2811LedStripStartTransfer(void)
{
    ws2811CleanDmaBuffer();

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_ClearFlag(dmaRef, DMA_IT_TCIF | DMA_IT_TEIF | DMA_IT_DMEIF | DMA_IT_FEIF);
    xDMA_MemoryTargetConfig(dmaRef, (uint32_t)ledStripDMABuffer, 0);
    xDMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);

    TIM_SetCnt(timer, 0);
    TIM_EnableDma(timer, timerDmaRequest, ENABLE);
    __DSB();
    xDMA_Cmd(dmaRef, ENABLE);
}

#endif
