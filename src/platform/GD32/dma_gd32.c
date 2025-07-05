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

#ifdef USE_DMA

#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "platform/rcc.h"
#include "drivers/resource.h"

/*
 * DMA descriptors.
 */
dmaChannelDescriptor_t dmaDescriptors[DMA_LAST_HANDLER] = {
    DEFINE_DMA_CHANNEL(DMA0, 0,  0),
    DEFINE_DMA_CHANNEL(DMA0, 1,  6),
    DEFINE_DMA_CHANNEL(DMA0, 2, 16),
    DEFINE_DMA_CHANNEL(DMA0, 3, 22),
    DEFINE_DMA_CHANNEL(DMA0, 4, 32),
    DEFINE_DMA_CHANNEL(DMA0, 5, 38),
    DEFINE_DMA_CHANNEL(DMA0, 6, 48),
    DEFINE_DMA_CHANNEL(DMA0, 7, 54),

    DEFINE_DMA_CHANNEL(DMA1, 0,  0),
    DEFINE_DMA_CHANNEL(DMA1, 1,  6),
    DEFINE_DMA_CHANNEL(DMA1, 2, 16),
    DEFINE_DMA_CHANNEL(DMA1, 3, 22),
    DEFINE_DMA_CHANNEL(DMA1, 4, 32),
    DEFINE_DMA_CHANNEL(DMA1, 5, 38),
    DEFINE_DMA_CHANNEL(DMA1, 6, 48),
    DEFINE_DMA_CHANNEL(DMA1, 7, 54),
};

/*
 * DMA IRQ Handlers
 */
DEFINE_DMA_IRQ_HANDLER(0, 0, DMA0_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 1, DMA0_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 2, DMA0_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 3, DMA0_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 4, DMA0_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 5, DMA0_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 6, DMA0_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(0, 7, DMA0_CH7_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_CH0_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_CH1_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 2, DMA1_CH2_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 3, DMA1_CH3_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 4, DMA1_CH4_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 5, DMA1_CH5_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 6, DMA1_CH6_HANDLER)
DEFINE_DMA_IRQ_HANDLER(1, 7, DMA1_CH7_HANDLER)

#define DMA_RCU(x) ((x) == (void *)DMA0 ? RCC_AHB1(DMA0) : RCC_AHB1(DMA1))
void dmaEnable(dmaIdentifier_e identifier)
{
    (void) identifier;
    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);
    RCC_ClockCmd(DMA_RCU(dmaDescriptors[index].dma), ENABLE);
}

#define RETURN_TCIF_FLAG(s, n) if (s == DMA0_CH ## n ## _BASE || s == DMA1_CH ## n ## _BASE) return DMA_IT_TCIF ## n

static int gd32_dma_channel_get(uint32_t dma_chan_base, uint32_t dma_periph)
{
    uint32_t chan_base = (dma_chan_base - dma_periph);
    int channel = DMA_CH0;

    switch(chan_base) {
    case 0x10:
        channel = DMA_CH0;
        break;
    case 0x28:
        channel = DMA_CH1;
        break;
    case 0x40:
        channel = DMA_CH2;
        break;
    case 0x58:
        channel = DMA_CH3;
        break;
    case 0x70:
        channel = DMA_CH4;
        break;
    case 0x88:
        channel = DMA_CH5;
        break;
    case 0xA0:
        channel = DMA_CH6;
        break;
    case 0xB8:
        channel = DMA_CH7;
        break;
    default:
        break;
    }
    return channel;
}

static uint32_t dma_int_ftf_flag_get(uint32_t dma_chan_base)
{
    uint32_t status = 0;
    uint32_t dma_periph;
    int channel;

    if(DMA1 > dma_chan_base) {
        dma_periph = DMA0;
    } else {
        dma_periph = DMA1;
    }

    channel = gd32_dma_channel_get(dma_chan_base, dma_periph);

    status = dma_interrupt_flag_get(dma_periph, channel ,DMA_INT_FLAG_FTF);

    return status;
}

uint32_t dma_enable_status_get(uint32_t dma_chan_base)
{
    uint32_t dma_periph;
    uint32_t status;
    int channel;

    if(DMA1 > dma_chan_base) {
        dma_periph = DMA0;
    } else {
        dma_periph = DMA1;
    }

    channel = gd32_dma_channel_get(dma_chan_base, dma_periph);

    status = (DMA_CHCTL(dma_periph, channel) & DMA_CHXCTL_CHEN);

    return status;
}

void dmaSetHandler(dmaIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback, uint32_t priority, uint32_t userParam)
{

    const int index = DMA_IDENTIFIER_TO_INDEX(identifier);

    RCC_ClockCmd(DMA_RCU(dmaDescriptors[index].dma), ENABLE);
    dmaDescriptors[index].irqHandlerCallback = callback;
    dmaDescriptors[index].userParam = userParam;
    dmaDescriptors[index].completeFlag = dma_int_ftf_flag_get((uint32_t)dmaDescriptors[index].ref);

    nvic_irq_enable(dmaDescriptors[index].irqN, NVIC_PRIORITY_BASE(priority), NVIC_PRIORITY_SUB(priority));
}


void gd32_dma_chbase_parse(uint32_t dma_chan_base, uint32_t *dma_periph, int *dma_channel)
{
    if(DMA1 > dma_chan_base) {
        *dma_periph = DMA0;
    } else {
        *dma_periph = DMA1;
    }

    *dma_channel = (int)(dma_chan_base - (*dma_periph) - 0x10)/0x18;
}

void gd32_dma_init(uint32_t dma_chan_base, dma_single_data_parameter_struct *init_struct)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    dma_single_data_mode_init(dma_periph, channel, init_struct);
}

void gd32_dma_general_init(uint32_t dma_chan_base, dma_general_config_struct *init_struct)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    if(DMA_DATA_MODE_MULTI == init_struct->data_mode) {
        dma_multi_data_mode_init(dma_periph, channel, (dma_multi_data_parameter_struct *)&init_struct->config.init_struct_m);
    } else {
        dma_single_data_mode_init(dma_periph, channel, (dma_single_data_parameter_struct *)&init_struct->config.init_struct_s);
    }

    dma_channel_subperipheral_select(dma_periph, channel, init_struct->sub_periph);
}

void gd32_dma_deinit(uint32_t dma_chan_base)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    dma_deinit(dma_periph, channel);
}

void gd32_dma_channel_state_config(uint32_t dma_chan_base, ControlStatus new_state)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    if(ENABLE == new_state) {
        dma_flag_clear(dma_periph, channel, DMA_FLAG_FTF);
        dma_flag_clear(dma_periph, channel, DMA_FLAG_HTF);
        dma_channel_enable(dma_periph, channel);
    } else {
        dma_channel_disable(dma_periph, channel);
    }
}

void gd32_dma_int_config(uint32_t dma_chan_base, uint32_t source, ControlStatus new_state)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    if(ENABLE == new_state) {
        dma_interrupt_enable(dma_periph, channel, source);
    } else {
        dma_interrupt_disable(dma_periph, channel, source);
    }
}

void gd32_dma_transnum_config(uint32_t dma_chan_base, uint32_t number)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    dma_transfer_number_config(dma_periph, channel, number);
}

uint16_t gd32_dma_transnum_get(uint32_t dma_chan_base)
{
    uint32_t dma_periph ;
    uint16_t number;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    number = (uint16_t)dma_transfer_number_get(dma_periph, channel);
    return number;
}

FlagStatus gd32_dma_flags_get(uint32_t dma_chan_base, uint32_t flag)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    return dma_flag_get(dma_periph, channel, flag);
}

void gd32_dma_flag_clear(uint32_t dma_chan_base, uint32_t flag)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);

    dma_flag_clear(dma_periph, channel, flag);
}

void gd32_dma_memory_addr_config(uint32_t dma_chan_base, uint32_t address, uint8_t memory_flag)
{
    uint32_t dma_periph ;
    int channel;

    gd32_dma_chbase_parse(dma_chan_base, &dma_periph, &channel);
 
    dma_memory_address_config(dma_periph, channel, memory_flag, address);
}

#endif
