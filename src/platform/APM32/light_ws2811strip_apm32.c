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

#include "platform.h"

#ifdef USE_LED_STRIP

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/system.h"
#include "drivers/timer.h"

#include "drivers/light_ws2811strip.h"
#include "platform/light_ws2811strip_stm32.h"

static IO_t ws2811IO = IO_NONE;

static TMR_HandleTypeDef TmrHandle;
static uint16_t timerChannel = 0;

static FAST_IRQ_HANDLER void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    DAL_DMA_IRQHandler(TmrHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TmrHandle, timerChannel, DISABLE);
    ws2811LedDataTransferInProgress = false;
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

    TMR_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;

    dmaResource_t *dmaRef;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }
    TmrHandle.Instance = timer;

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    TmrHandle.Init.Prescaler = prescaler;
    TmrHandle.Init.Period = period; // 800kHz
    TmrHandle.Init.ClockDivision = TMR_CLOCKDIVISION_DIV1;
    TmrHandle.Init.CounterMode = TMR_COUNTERMODE_UP;
    TmrHandle.Init.AutoReloadPreload = TMR_AUTORELOAD_PRELOAD_ENABLE;
    if (DAL_TMR_PWM_Init(&TmrHandle) != DAL_OK) {
        /* Initialization Error */
        return false;
    }

    static DMA_HandleTypeDef hdma_tim;

    ws2811IO = IOGetByTag(ledStripIoTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

    __DAL_RCM_DMA1_CLK_ENABLE();
    __DAL_RCM_DMA2_CLK_ENABLE();

    /* Set the parameters to be configured */
    hdma_tim.Init.Channel = dmaChannel;
    hdma_tim.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim.Init.Mode = DMA_NORMAL;
    hdma_tim.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* Set hdma_tim instance */
    hdma_tim.Instance = (DMA_ARCH_TYPE *)dmaRef;

    uint16_t dmaIndex = timerDmaIndex(timerChannel);

    /* Link hdma_tim to hdma[x] (channelx) */
    __DAL_LINKDMA(&TmrHandle, hdma[dmaIndex], hdma_tim);

    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (DAL_DMA_Init(TmrHandle.hdma[dmaIndex]) != DAL_OK) {
        /* Initialization Error */
        return false;
    }

    TMR_OC_InitTypeDef TMR_OCInitStructure;

    /* PWM1 Mode configuration: Channel1 */
    TMR_OCInitStructure.OCMode = TMR_OCMODE_PWM1;
    TMR_OCInitStructure.OCIdleState = TMR_OCIDLESTATE_SET;
    TMR_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OCPOLARITY_LOW : TMR_OCPOLARITY_HIGH;
    TMR_OCInitStructure.OCNIdleState = TMR_OCNIDLESTATE_RESET;
    TMR_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OCNPOLARITY_LOW : TMR_OCNPOLARITY_HIGH;
    TMR_OCInitStructure.Pulse = 0;
    TMR_OCInitStructure.OCFastMode = TMR_OCFAST_DISABLE;
    if (DAL_TMR_PWM_ConfigChannel(&TmrHandle, &TMR_OCInitStructure, timerChannel) != DAL_OK) {
        /* Configuration Error */
        return false;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (DAL_TMREx_PWMN_Start(&TmrHandle, timerChannel) != DAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    } else {
        if (DAL_TMR_PWM_Start(&TmrHandle, timerChannel) != DAL_OK) {
            /* Starting PWM generation Error */
            return false;
        }
    }

    return true;
}

void ws2811LedStripStartTransfer(void)
{
    if (DMA_SetCurrDataCounter(&TmrHandle, timerChannel, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE) != DAL_OK) {
        /* DMA set error */
        ws2811LedDataTransferInProgress = false;
        return;
    }
    /* Reset timer counter */
    __DAL_TMR_SET_COUNTER(&TmrHandle,0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TmrHandle,timerChannel,ENABLE);
}

#endif // USE_LED_STRIP
