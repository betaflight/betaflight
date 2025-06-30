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

typedef enum {
    DMA_INVALID = -1,
    DMA_NONE = 0,
    DMA_FIRST_HANDLER = 1,
    DMA_CH0_HANDLER = DMA_FIRST_HANDLER,
    DMA_CH1_HANDLER,
    DMA_CH2_HANDLER,
    DMA_CH3_HANDLER,
    DMA_CH4_HANDLER,
    DMA_CH5_HANDLER,
    DMA_CH6_HANDLER,
    DMA_CH7_HANDLER,
    DMA_CH8_HANDLER,
    DMA_CH9_HANDLER,
    DMA_CH10_HANDLER,
    DMA_CH11_HANDLER,
#if defined(RP2350A) || defined(RP2350B)
    DMA_CH12_HANDLER,
    DMA_CH13_HANDLER,
    DMA_CH14_HANDLER,
    DMA_CH15_HANDLER,
    DMA_LAST_HANDLER = DMA_CH15_HANDLER
#else
    DMA_LAST_HANDLER = DMA_CH11_HANDLER
#endif
} dmaIdentifier_e;

#define DMA_DEVICE_NO(x)    (0)
#define DMA_DEVICE_INDEX(x) ((x)-1)
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(c) { \
    .dma = NULL, \
    .ref = NULL, \
    .channel = c-1, \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = 0, \
    .userParam = 0, \
    .owner.owner = 0, \
    .owner.resourceIndex = 0 \
    }

#define DMA_IDENTIFIER_TO_CHANNEL(identifier) ((identifier) - DMA_FIRST_HANDLER)
#define DMA_CHANNEL_TO_IDENTIFIER(channel) ((dmaIdentifier_e)((channel) + DMA_FIRST_HANDLER))
#define DMA_CHANNEL_TO_INDEX(channel) (channel)

dmaIdentifier_e dmaGetFreeIdentifier(void);
