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

#pragma once

#include "platform.h"
#include "drivers/resource.h"

#if defined(GD32F4)
#define PLATFORM_TRAIT_DMA_STREAM_REQUIRED 1
#endif

typedef enum {
    DMA_NONE = 0,
    DMA_FIRST_HANDLER = 1,
    DMA0_CH0_HANDLER = DMA_FIRST_HANDLER,
    DMA0_CH1_HANDLER,
    DMA0_CH2_HANDLER,
    DMA0_CH3_HANDLER,
    DMA0_CH4_HANDLER,
    DMA0_CH5_HANDLER,
    DMA0_CH6_HANDLER,
    DMA0_CH7_HANDLER,
    DMA1_CH0_HANDLER,
    DMA1_CH1_HANDLER,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA_LAST_HANDLER = DMA1_CH7_HANDLER
} dmaIdentifier_e;

/* DMA general initialize struct */
typedef struct
{
    uint32_t single_mode;                               /* single or multiple mode */
    int sub_periph;                                     /* specify DMA channel peripheral */
    void * general_init_s;                              /* single or multiple struct */
} dma_general_init_struct;


uint32_t dmaGetChannel(const uint8_t channel);

#define DMA_DEVICE_NO(x)    ((((x)-1) / 8) + 1)
#define DMA_DEVICE_INDEX(x) ((((x)-1) % 8))
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(d, s, f) { \
    .dma = (void *)d, \
    .ref = (dmaResource_t *)d ## _CH ## s ## _BASE, \
    .stream = s, \
    .irqHandlerCallback = NULL, \
    .flagsShift = f, \
    .irqN = d ## _Channel ## s ## _IRQn, \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0 \
    }

#define DEFINE_DMA_IRQ_HANDLER(d, s, i) FAST_IRQ_HANDLER void DMA ## d ## _Channel ## s ## _IRQHandler(void) {\
                                                                const uint8_t index = DMA_IDENTIFIER_TO_INDEX(i); \
                                                                dmaCallbackHandlerFuncPtr handler = dmaDescriptors[index].irqHandlerCallback; \
                                                                if (handler) \
                                                                    handler(&dmaDescriptors[index]); \
                                                            }

#define DMA_CLEAR_FLAG(d, flag) if (d->flagsShift > 31) DMA_INTC1((uint32_t)d->dma) = (flag << (d->flagsShift - 32)); else DMA_INTC0((uint32_t)d->dma) = (flag << d->flagsShift)
#define DMA_GET_FLAG_STATUS(d, flag) (d->flagsShift > 31 ? DMA_INTF1((uint32_t)d->dma) & (flag << (d->flagsShift - 32)): DMA_INTF0((uint32_t)d->dma) & (flag << d->flagsShift))

extern void gd32_dma_init(uint32_t dma_chan_base, dma_single_data_parameter_struct *init_struct);
extern void gd32_dma_general_init(uint32_t dma_chan_base, dma_general_config_struct *init_struct);
extern void gd32_dma_deinit(uint32_t dma_chan_base);
extern void gd32_dma_channel_state_config(uint32_t dma_chan_base, ControlStatus new_state);
extern void gd32_dma_int_config(uint32_t dma_chan_base, uint32_t source, ControlStatus new_state);
extern void gd32_dma_transnum_config(uint32_t dma_chan_base, uint32_t number);
extern uint16_t gd32_dma_transnum_get(uint32_t dma_chan_base);
extern FlagStatus gd32_dma_flag_get(uint32_t dma_chan_base, uint32_t flag);
extern void gd32_dma_flag_clear(uint32_t dma_chan_base, uint32_t flag);
extern void gd32_dma_chbase_parse(uint32_t dma_chan_base, uint32_t *dma_periph, int *dma_channel);
extern void gd32_dma_memory_addr_config(uint32_t dma_chan_base, uint32_t address, uint8_t memory_flag);

#define xDMA_Init(dmaResource, initStruct) gd32_dma_general_init((uint32_t)dmaResource, initStruct)
#define xDMA_DeInit(dmaResource) gd32_dma_deinit((uint32_t)dmaResource)
#define xDMA_Cmd(dmaResource, newState) gd32_dma_channel_state_config((uint32_t)dmaResource, newState)
#define xDMA_ITConfig(dmaResource, flags, newState) gd32_dma_int_config((uint32_t)(dmaResource), flags, newState)
#define xDMA_GetCurrDataCounter(dmaResource) gd32_dma_transnum_get((uint32_t)(dmaResource))
#define xDMA_SetCurrDataCounter(dmaResource, count) gd32_dma_transnum_config((uint32_t)(dmaResource), count)
#define xDMA_GetFlagStatus(dmaResource, flags) gd32_dma_flag_get((uint32_t)(dmaResource), flags)
#define xDMA_ClearFlag(dmaResource, flags) gd32_dma_flag_clear((uint32_t)(dmaResource), flags)
#define xDMA_MemoryTargetConfig(dmaResource, address, target) gd32_dma_memory_addr_config((uint32_t)(dmaResource), address, target)

extern uint32_t dma_enable_status_get(uint32_t dma_chan_base);
#define IS_DMA_ENABLED(reg) (dma_enable_status_get((uint32_t)reg))
