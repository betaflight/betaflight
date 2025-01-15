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

#ifdef USE_TRANSPONDER

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir_arcitimer.h"
#include "drivers/transponder_ir_erlt.h"
#include "drivers/transponder_ir_ilap.h"

#include "drivers/transponder_ir.h"

volatile uint8_t transponderIrDataTransferInProgress = 0;

static IO_t transponderIO = IO_NONE;
static TMR_HandleTypeDef TmrHandle;
static uint16_t timerChannel = 0;
static uint8_t output;
static uint8_t alternateFunction;

transponder_t transponder;

bool transponderInitialised = false;

FAST_IRQ_HANDLER static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    DAL_DMA_IRQHandler(TmrHandle.hdma[descriptor->userParam]);
    TIM_DMACmd(&TmrHandle, timerChannel, DISABLE);
    transponderIrDataTransferInProgress = 0;
}

void transponderIrHardwareInit(ioTag_t ioTag, transponder_t *transponder)
{
    if (!ioTag) {
        return;
    }

    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_TRANSPONDER, 0);
    TMR_TypeDef *timer = timerHardware->tim;
    timerChannel = timerHardware->channel;
    output = timerHardware->output;
    alternateFunction = timerHardware->alternateFunction;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return;
    }

    dmaResource_t *dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaResource_t *dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    dmaIdentifier_e dmaIdentifier = dmaGetIdentifier(dmaRef);
    if (dmaRef == NULL || !dmaAllocate(dmaIdentifier, OWNER_TRANSPONDER, 0)) {
        return;
    }

    /* Time base configuration */

    TmrHandle.Instance = timer;

    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, transponder->timer_hz);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, transponder->timer_carrier_hz);

    transponder->bitToggleOne = period / 2;

    TmrHandle.Init.Prescaler = prescaler;
    TmrHandle.Init.Period = period; // 800kHz
    TmrHandle.Init.ClockDivision = TMR_CLOCKDIVISION_DIV1;
    TmrHandle.Init.CounterMode = TMR_COUNTERMODE_UP;
    if (DAL_TMR_PWM_Init(&TmrHandle) != DAL_OK) {
        /* Initialization Error */
        return;
    }

    /* IO configuration */

    static DMA_HandleTypeDef hdma_tim;

    transponderIO = IOGetByTag(ioTag);
    IOInit(transponderIO, OWNER_TRANSPONDER, 0);
    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), timerHardware->alternateFunction);

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

    dmaEnable(dmaIdentifier);
    dmaSetHandler(dmaIdentifier, TRANSPONDER_DMA_IRQHandler, NVIC_PRIO_TRANSPONDER_DMA, dmaIndex);

    /* Initialize TIMx DMA handle */
    if (DAL_DMA_Init(TmrHandle.hdma[dmaIndex]) != DAL_OK) {
        /* Initialization Error */
        return;
    }

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    /* PWM1 Mode configuration: Channel1 */
    TMR_OC_InitTypeDef  TMR_OCInitStructure;

    TMR_OCInitStructure.OCMode = TMR_OCMODE_PWM1;
    TMR_OCInitStructure.OCIdleState = TMR_OCIDLESTATE_RESET;
    TMR_OCInitStructure.OCPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OCPOLARITY_LOW : TMR_OCPOLARITY_HIGH;
    TMR_OCInitStructure.OCNIdleState = TMR_OCNIDLESTATE_RESET;
    TMR_OCInitStructure.OCNPolarity = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OCNPOLARITY_LOW : TMR_OCNPOLARITY_HIGH;
    TMR_OCInitStructure.Pulse = 0;
    TMR_OCInitStructure.OCFastMode = TMR_OCFAST_DISABLE;
    if (DAL_TMR_PWM_ConfigChannel(&TmrHandle, &TMR_OCInitStructure, timerChannel) != DAL_OK) {
        /* Configuration Error */
        return;
    }
    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        if (DAL_TMREx_PWMN_Start(&TmrHandle, timerChannel) != DAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    } else {
        if (DAL_TMR_PWM_Start(&TmrHandle, timerChannel) != DAL_OK) {
            /* Starting PWM generation Error */
            return;
        }
    }

    transponderInitialised = true;
}

bool transponderIrInit(const ioTag_t ioTag, const transponderProvider_e provider)
{
    if (!ioTag) {
        return false;
    }

    switch (provider) {
        case TRANSPONDER_ARCITIMER:
            transponderIrInitArcitimer(&transponder);
            break;
        case TRANSPONDER_ILAP:
            transponderIrInitIlap(&transponder);
            break;
        case TRANSPONDER_ERLT:
            transponderIrInitERLT(&transponder);
            break;
        default:
            return false;
    }

    transponderIrHardwareInit(ioTag, &transponder);

    return true;
}

bool isTransponderIrReady(void)
{
    return !transponderIrDataTransferInProgress;
}

void transponderIrWaitForTransmitComplete(void)
{
#ifdef DEBUG
    static uint32_t waitCounter = 0;
#endif

    while (transponderIrDataTransferInProgress) {
#ifdef DEBUG
        waitCounter++;
#endif
    }
}

void transponderIrUpdateData(const uint8_t* transponderData)
{
     transponderIrWaitForTransmitComplete();
     transponder.vTable->updateTransponderDMABuffer(&transponder, transponderData);
}

void transponderIrDMAEnable(transponder_t *transponder)
{
    if (!transponderInitialised) {
        return;
    }

    if (DMA_SetCurrDataCounter(&TmrHandle, timerChannel, transponder->transponderIrDMABuffer.ilap, transponder->dma_buffer_size) != DAL_OK) {
        /* DMA set error */
        transponderIrDataTransferInProgress = 0;
        return;
    }

    /* Reset timer counter */
    __DAL_TMR_SET_COUNTER(&TmrHandle, 0);
    /* Enable channel DMA requests */
    TIM_DMACmd(&TmrHandle, timerChannel, ENABLE);
}

void transponderIrDisable(void)
{
    if (!transponderInitialised) {
        return;
    }

    TIM_DMACmd(&TmrHandle, timerChannel, DISABLE);
    if (output & TIMER_OUTPUT_N_CHANNEL) {
        DAL_TMREx_PWMN_Stop(&TmrHandle, timerChannel);
    } else {
        DAL_TMR_PWM_Stop(&TmrHandle, timerChannel);
    }

    IOInit(transponderIO, OWNER_TRANSPONDER, 0);

#ifdef TRANSPONDER_INVERTED
    IOHi(transponderIO);
#else
    IOLo(transponderIO);
#endif

    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_PULLDOWN), alternateFunction);
}

void transponderIrTransmit(void)
{
    transponderIrWaitForTransmitComplete();

    transponderIrDataTransferInProgress = 1;
    transponderIrDMAEnable(&transponder);
}
#endif // USE_TRANSPONDER
