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

#include "build/debug.h"

#include "common/color.h"

#include "drivers/dma.h"
#include "drivers/dma_reqmap.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"
#include "drivers/timer.h"

#include "drivers/light_ws2811strip.h"

static IO_t ws2811IO = IO_NONE;
#if defined(GD32F4)
static dmaResource_t *dmaRef = NULL;
#else
#error "No MCU definition in light_ws2811strip_stdperiph.c"
#endif
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
#if defined(USE_WS2811_SINGLE_COLOUR)
    static uint32_t counter = 0;
#endif

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_INT_FLAG_FTF)) {   
#if defined(USE_WS2811_SINGLE_COLOUR)
        counter++;
        if (counter == WS2811_LED_STRIP_LENGTH) {
            // Output low for 50us delay
            memset(ledStripDMABuffer, 0, sizeof(ledStripDMABuffer));
        } else if (counter == (WS2811_LED_STRIP_LENGTH + WS2811_DELAY_ITERATIONS)) {
            counter = 0;
            ws2811LedDataTransferInProgress = false;
            xDMA_Cmd(descriptor->ref, DISABLE);
        }
#else
        ws2811LedDataTransferInProgress = false;
        xDMA_Cmd(descriptor->ref, DISABLE);
#endif

        DMA_CLEAR_FLAG(descriptor, DMA_INT_FLAG_FTF);  
    }
}

bool ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return false;
    }

    timer_parameter_struct timer_initpara;
    timer_oc_parameter_struct timer_ocintpara;
    dma_single_data_parameter_struct dma_init_struct;

    const timerHardware_t *timerHardware = timerAllocate(ioTag, OWNER_LED_STRIP, 0);

    if (timerHardware == NULL) {
        return false;
    }

    timer = timerHardware->tim;

#if defined(USE_DMA_SPEC)
    const dmaChannelSpec_t *dmaSpec = dmaGetChannelSpecByTimer(timerHardware);

    if (dmaSpec == NULL) {
        return false;
    }

    dmaRef = dmaSpec->ref;
#if defined(GD32F4)
    uint32_t dmaChannel = dmaSpec->channel;
#endif
#else
    dmaRef = timerHardware->dmaRef;
#if defined(GD32F4)
    uint32_t dmaChannel = timerHardware->dmaChannel;
#endif
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP), timerHardware->alternateFunction);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    // Stop timer
    timer_disable((uint32_t)timer);

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* Time base configuration */
    timer_struct_para_init(&timer_initpara);
    timer_initpara.period            = period;     // 800kHz
    timer_initpara.prescaler         = prescaler;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.repetitioncounter = 0;
    timer_init((uint32_t)timer, &timer_initpara);

    /* PWM1 Mode configuration */
    timer_channel_output_struct_para_init(&timer_ocintpara);
    timer_channel_output_mode_config((uint32_t)timer, timerHardware->channel, TIMER_OC_MODE_PWM0);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
        timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
        timer_ocintpara.ocnpolarity  = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIMER_OCN_IDLE_STATE_LOW : TIMER_OCN_POLARITY_HIGH;
    } else {
        timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
        timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
        timer_ocintpara.ocpolarity  = (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TIMER_OC_IDLE_STATE_LOW : TIMER_OC_POLARITY_HIGH;
    }

    timer_channel_output_pulse_value_config((uint32_t)timer, timerHardware->channel, 0);

    timerOCInit(timer, timerHardware->channel, &timer_ocintpara);
    timerOCModeConfig(timer, timerHardware->channel, TIMER_OC_MODE_PWM0);
    timerOCPreloadConfig(timer, timerHardware->channel, TIMER_OC_SHADOW_ENABLE);

    timer_primary_output_config((uint32_t)timer, ENABLE);
    timer_auto_reload_shadow_enable((uint32_t)timer);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        timer_channel_complementary_output_state_config((uint32_t)timer, timerHardware->channel, TIMER_CCXN_ENABLE);
    } else {
        timer_channel_output_state_config((uint32_t)timer, timerHardware->channel, TIMER_CCX_ENABLE);
    }

    timer_enable((uint32_t)timer);

    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    xDMA_DeInit(dmaRef);

    /* configure DMA */
    xDMA_Cmd(dmaRef, DISABLE);
    xDMA_DeInit(dmaRef);
    dma_single_data_para_struct_init(&dma_init_struct);
    dma_init_struct.periph_addr = (uint32_t)timerCCR(timer, timerHardware->channel);
    dma_init_struct.number = WS2811_DMA_BUFFER_SIZE;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;

#if defined(GD32F4)

    uint32_t temp_dma_periph;
    int temp_dma_channel;

    gd32_dma_chbase_parse((uint32_t)dmaRef, &temp_dma_periph, &temp_dma_channel);

    dma_channel_subperipheral_select(temp_dma_periph, temp_dma_channel, dmaChannel);

    dma_init_struct.memory0_addr = (uint32_t)ledStripDMABuffer;
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
    dma_init_struct.periph_memory_width = DMA_PERIPH_WIDTH_32BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
#endif

#if defined(USE_WS2811_SINGLE_COLOUR)
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_ENABLE; 
#else
    dma_init_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE; 
#endif

    gd32_dma_init((uint32_t)dmaRef, &dma_init_struct);
    timer_dma_enable((uint32_t)timer, timerDmaSource(timerHardware->channel));
    xDMA_ITConfig(dmaRef, DMA_INT_FTF, ENABLE);    

    return true;
}

void ws2811LedStripDMAEnable(void)
{
    xDMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    timer_counter_value_config((uint32_t)timer, 0);
    timer_enable((uint32_t)timer);
    xDMA_Cmd(dmaRef, ENABLE);
}
#endif
