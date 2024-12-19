/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
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
#include "drivers/rcc.h"
#include "drivers/timer.h"

#include "drivers/light_ws2811strip.h"

static IO_t ws2811IO = IO_NONE;

static dmaResource_t *dmaRef = NULL;
static tmr_type *timer = NULL;

static void WS2811_DMA_IRQHandler(dmaChannelDescriptor_t *descriptor)
{
#if defined(USE_WS2811_SINGLE_COLOUR)
    static uint32_t counter = 0;
#endif

    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
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
        xDMA_Cmd(descriptor->ref, FALSE);
#endif

        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

bool ws2811LedStripHardwareInit(ioTag_t ioTag)
{
    if (!ioTag) {
        return false;
    }

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

#else
    dmaRef = timerHardware->dmaRef;
#endif

    if (dmaRef == NULL || !dmaAllocate(dmaGetIdentifier(dmaRef), OWNER_LED_STRIP, 0)) {
        return false;
    }

    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0);

    IOConfigGPIOAF(ws2811IO, IOCFG_AF_PP, timerHardware->alternateFunction);

    RCC_ClockCmd(timerRCC(timer), ENABLE);

    // Stop timer
    tmr_counter_enable(timer, FALSE);

    /* Compute the prescaler value */
    uint16_t prescaler = timerGetPrescalerByDesiredMhz(timer, WS2811_TIMER_MHZ);
    uint16_t period = timerGetPeriodByPrescaler(timer, prescaler, WS2811_CARRIER_HZ);

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* Time base configuration */
    tmr_base_init(timer,period-1,prescaler-1);
    tmr_clock_source_div_set(timer,TMR_CLOCK_DIV1);
    tmr_cnt_dir_set(timer,TMR_COUNT_UP);

    tmr_output_config_type  tmr_OCInitStruct;
    tmr_output_default_para_init(&tmr_OCInitStruct);
    tmr_OCInitStruct.oc_mode= TMR_OUTPUT_CONTROL_PWM_MODE_A;

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        tmr_OCInitStruct.occ_output_state = TRUE;
        tmr_OCInitStruct.occ_idle_state = FALSE;
        tmr_OCInitStruct.occ_polarity =  (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    } else {
        tmr_OCInitStruct.oc_output_state = TRUE;
        tmr_OCInitStruct.oc_idle_state = TRUE;
        tmr_OCInitStruct.oc_polarity =  (timerHardware->output & TIMER_OUTPUT_INVERTED) ? TMR_OUTPUT_ACTIVE_LOW : TMR_OUTPUT_ACTIVE_HIGH;
    }

    tmr_channel_value_set(timer, (timerHardware->channel-1)*2, 0);
    tmr_output_channel_config(timer,(timerHardware->channel-1)*2, &tmr_OCInitStruct);

    tmr_period_buffer_enable(timer, TRUE);
    tmr_output_channel_buffer_enable(timer, ((timerHardware->channel-1) * 2), TRUE);

    tmr_dma_request_enable(timer, timerDmaSource(timerHardware->channel), TRUE);

    if (timerHardware->output & TIMER_OUTPUT_N_CHANNEL) {
        tmr_channel_enable(timer, ((timerHardware->channel -1) * 2 + 1), TRUE);
    } else {
        tmr_channel_enable(timer, ((timerHardware->channel-1) * 2), TRUE);
    }

    dmaEnable(dmaGetIdentifier(dmaRef));
    dmaSetHandler(dmaGetIdentifier(dmaRef), WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);

    xDMA_Cmd(dmaRef, FALSE);
    xDMA_DeInit(dmaRef);

    dma_init_type dma_init_struct;
    dma_init_struct.peripheral_base_addr = (uint32_t)timerCCR(timer, timerHardware->channel);
    dma_init_struct.buffer_size = WS2811_DMA_BUFFER_SIZE;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)ledStripDMABuffer;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;

#if defined(USE_WS2811_SINGLE_COLOUR)
    dma_init_struct.loop_mode_enable = TRUE;
#else
    dma_init_struct.loop_mode_enable = FALSE;
#endif

   xDMA_Init(dmaRef, &dma_init_struct);

#if defined(USE_DMA_SPEC)
    dmaMuxEnable(dmaGetIdentifier(dmaRef), dmaSpec->dmaMuxId);
#else
    #warning "USE_DMA_SPEC DETECT FAIL IN LEDSTRIP AT32F43X NEED DMA MUX "
#endif

    xDMA_ITConfig(dmaRef, DMA_IT_TCIF, TRUE);
    xDMA_Cmd(dmaRef, TRUE);

    tmr_output_enable(timer, TRUE);
    tmr_counter_enable(timer, TRUE);

    return true;
}

void ws2811LedStripDMAEnable(void)
{
    xDMA_SetCurrDataCounter(dmaRef, WS2811_DMA_BUFFER_SIZE);
    tmr_counter_value_set(timer, 0);
    tmr_counter_enable(timer, TRUE);
    xDMA_Cmd(dmaRef, TRUE);

}
#endif
