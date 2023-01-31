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

#pragma once

#include "platform.h"
#include "drivers/resource.h"

typedef enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA2_CH1_HANDLER,
    DMA2_CH2_HANDLER,
    DMA2_CH3_HANDLER,
    DMA2_CH4_HANDLER,
    DMA2_CH5_HANDLER,
    DMA2_CH6_HANDLER,
    DMA2_CH7_HANDLER,
    DMA_LAST_HANDLER = DMA2_CH7_HANDLER
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    ((((x)-1) / 7) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 7) + 1)

uint32_t dmaGetChannel(const uint8_t channel);

#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, c, f) { \
    .dma = d, \
    .ref = (dmaResource_t *)d ## _CHANNEL ## c ##_BASE, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## c ## _IRQn, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 ,\
    .dmamux= (dmamux_channel_type *) d ## MUX_CHANNEL ##c \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, c, i) DMA_HANDLER_CODE void DMA ## d ## _Channel ## c ## _IRQHandler(void) {\
                                                                        const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                        dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                        if (handler) \
                                                                            handler(&dmaDescriptors[index]); \
                                                                    }

#define DMA_CLEAR_FLAG(d, flag) d->dma->clr = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->dma->sts & (flag << d->flagsShift))
#define DMA_IT_GLOB         ((uint32_t)0x00000001) // channel global interput flag
#define DMA_IT_TCIF         ((uint32_t)0x00000002) // channel full transport flag
#define DMA_IT_HTIF         ((uint32_t)0x00000004) // channel half transport flag
#define DMA_IT_TEIF         ((uint32_t)0x00000008) // channel transport error flag
#define DMA_IT_DMEIF        ((uint32_t)0x00000004) // at32 has no direct mode transfer mode
#define DMA_HANDLER_CODE

void dmaMuxEnable(dmaIdentifier_e identifier, uint32_t dmaMuxId);

#define xDMA_Init(dmaResource, initStruct) dma_init((DMA_ARCH_TYPE *)(dmaResource), initStruct)
#define xDMA_DeInit(dmaResource) dma_reset((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_Cmd(dmaResource, newState)  dma_channel_enable((DMA_ARCH_TYPE *)(dmaResource), newState)
#define xDMA_ITConfig(dmaResource, flags, newState) dma_interrupt_enable((DMA_ARCH_TYPE *)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) dma_data_number_get((DMA_ARCH_TYPE *)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) dma_data_number_set((DMA_ARCH_TYPE *)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) dma_flag_get((DMA_ARCH_TYPE *)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) dma_flag_clear((DMA_ARCH_TYPE *)(dmaResource), flags)
