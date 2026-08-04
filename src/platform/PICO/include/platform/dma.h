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

#include "drivers/dma.h"

#define DMA_INVALID (-1)
#define DMA_CH0_HANDLER (DMA_FIRST_HANDLER)
#define DMA_CH1_HANDLER (DMA_FIRST_HANDLER + 1)
#define DMA_CH2_HANDLER (DMA_FIRST_HANDLER + 2)
#define DMA_CH3_HANDLER (DMA_FIRST_HANDLER + 3)
#define DMA_CH4_HANDLER (DMA_FIRST_HANDLER + 4)
#define DMA_CH5_HANDLER (DMA_FIRST_HANDLER + 5)
#define DMA_CH6_HANDLER (DMA_FIRST_HANDLER + 6)
#define DMA_CH7_HANDLER (DMA_FIRST_HANDLER + 7)
#define DMA_CH8_HANDLER (DMA_FIRST_HANDLER + 8)
#define DMA_CH9_HANDLER (DMA_FIRST_HANDLER + 9)
#define DMA_CH10_HANDLER (DMA_FIRST_HANDLER + 10)
#define DMA_CH11_HANDLER (DMA_FIRST_HANDLER + 11)
#ifdef RP2350
#define DMA_CH12_HANDLER (DMA_FIRST_HANDLER + 12)
#define DMA_CH13_HANDLER (DMA_FIRST_HANDLER + 13)
#define DMA_CH14_HANDLER (DMA_FIRST_HANDLER + 14)
#define DMA_CH15_HANDLER (DMA_FIRST_HANDLER + 15)
#define DMA_LAST_HANDLER DMA_CH15_HANDLER
#else
#define DMA_LAST_HANDLER DMA_CH11_HANDLER
#endif

#define DMA_DEVICE_NO(x)    (0)
#define DMA_DEVICE_INDEX(x) ((x)-1)
#define DMA_OUTPUT_INDEX    0
#define DMA_OUTPUT_STRING   "DMA%d Channel %d:"
#define DMA_INPUT_STRING    "DMA%d_CH%d"

#define DEFINE_DMA_CHANNEL(c) { \
    .dma = NULL, \
    .ref = NULL, \
    .stream = 0, \
    .channel = c-1, \
    .irqHandlerCallback = NULL, \
    .flagsShift = 0, \
    .irqN = 0, \
    .userParam = 0, \
    .resourceOwner.owner = 0, \
    .resourceOwner.index = 0, \
    .dmamux = NULL \
    }

#define DMA_IDENTIFIER_TO_CHANNEL(identifier) ((identifier) - DMA_FIRST_HANDLER)
#define DMA_CHANNEL_TO_IDENTIFIER(channel) ((dmaIdentifier_e)((channel) + DMA_FIRST_HANDLER))
#define DMA_CHANNEL_TO_INDEX(channel) (channel)

dmaIdentifier_e dmaGetFreeIdentifier(void);
