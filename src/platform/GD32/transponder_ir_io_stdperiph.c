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
#include "platform/rcc.h"
#include "drivers/timer.h"
#include "drivers/transponder_ir_arcitimer.h"
#include "drivers/transponder_ir_erlt.h"
#include "drivers/transponder_ir_ilap.h"

#include "drivers/transponder_ir.h"

volatile uint8_t transponderIrDataTransferInProgress = 0;

static IO_t transponderIO = IO_NONE;
static TIM_TypeDef *timer = NULL;
uint8_t alternateFunction;
static dmaResource_t *dmaRef = NULL;

transponder_t transponder;

static void TRANSPONDER_DMA_IRQHandler(dmaChannelDescriptor_t* descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_INT_FLAG_FTF)) {
        transponderIrDataTransferInProgress = 0;

        xDMA_Cmd(descriptor->ref, DISABLE);
        DMA_CLEAR_FLAG(descriptor, DMA_INT_FLAG_FTF);
    }
}

void transponderIrHardwareInit(ioTag_t ioTag, transponder_t *transponder)
{
    (void) transponder;
    if (!ioTag) {
        return;
    }

    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;
    dma_single_data_parameter_struct dma_init_struct;

    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_TRANSPONDER, 0);
    timer = timerHardware->tim;
    alternateFunction = timerHardware->alternateFunction;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return;
    }

    dmaRef = dmaSpec->ref;
    uint32_t dmaChannel = dmaSpec->channel;
#else
    dmaRef = timerHardware->dmaRef;
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_TRANSPONDER, 0)) {
        return;
    }

    transponderIO = IOGetByTag(ioTag);
    IOInit(transponderIO, OWNER_TRANSPONDER, 0);

    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN), timerHardware->alternateFunction);
    
    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), TRANSPONDER_DMA_IRQHandler, NVIC_PRIO_TRANSPONDER_DMA, 0);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    if (transponder->timer_hz == 0 || transponder->timer_carrier_hz == 0) {
        return;
    }
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, transponder->timer_hz);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, transponder->timer_carrier_hz);

    transponder->bitToggleOne = period / 2;

    timer_struct_para_init(&timer_initpara);
    timer_initpara.period            = period;
    timer_initpara.prescaler         = prescaler;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.repetitioncounter = 0;
    timer_init((uint32_t)timer, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_channel_output_mode_config((uint32_t)timer, timerHardware->channel, TIMER_OC_MODE_PWM0);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
        timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
    } else {
        timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
        timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
    }

    timer_ocintpara.ocpolarity  = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIMER_OC_IDLE_STATE_LOW : TIMER_OC_POLARITY_HIGH;
    timer_channel_output_pulse_value_config((uint32_t)timer, timerHardware->channel, 0);

    timerOCInit(timer, timerHardware->channel, &timer_ocintpara);
    timerOCModeConfig(timer, timerHardware->channel, TIMER_OC_MODE_PWM0);
    timerOCPreloadConfig(timer, timerHardware->channel, TIMER_OC_SHADOW_ENABLE);
    timer_primary_output_config((uint32_t)timer, ENABLE);

    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);

    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr = (uint32_t)timerCCR(timer, timerHardware->channel);

    uint32_t temp_dma_periph;
    int temp_dma_channel;
    gd32_dma_chbase_parse((uint32_t)dmaRef, &temp_dma_periph, &temp_dma_channel);

    dma_channel_subperipheral_select(temp_dma_periph, temp_dma_channel, dmaChannel);

    dma_init_struct.memory0_addr =(uint32_t)&(transponder->transponderIrDMABuffer);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.number = transponder->dma_buffer_size;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;

#if defined(GD32F4)
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_32BIT; 
#endif

    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE; 
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    
    gd32_dma_init((uint32_t)dmaRef, &dma_init_struct);

    timer_dma_enable((uint32_t)timer, timerDmaSource(timerHardware->channel));

    xDMA_ITConfig(dmaRef, DMA_INT_FTF, ENABLE);
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
    xDMA_SetCurrDataCounter(dmaRef, transponder->dma_buffer_size);
    timer_counter_value_config((uint32_t)timer, 0);
    timer_enable((uint32_t)timer);
    xDMA_Cmd(dmaRef, ENABLE);
}

void transponderIrDisable(void)
{
    xDMA_Cmd(dmaRef, DISABLE);
    timer_disable((uint32_t)timer);

    IOInit(transponderIO, OWNER_TRANSPONDER, 0);

    IOConfigGPIOAF(transponderIO, IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN), alternateFunction);

#ifdef TRANSPONDER_INVERTED
    IOHi(transponderIO);
#else
    IOLo(transponderIO);
#endif
}

void transponderIrTransmit(void)
{
    transponderIrWaitForTransmitComplete();

    transponderIrDataTransferInProgress = 1;
    transponderIrDMAEnable(&transponder);
}
#endif
