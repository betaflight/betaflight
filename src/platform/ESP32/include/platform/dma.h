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

// ESP32-S3 has GDMA with 5 channels, each with TX and RX pairs.
// Each GDMA channel has an independent RX (in) and TX (out) sub-channel.
// For SPI DMA we allocate channel pairs: one for TX, one for RX.
#define DMA_INVALID (-1)
#define DMA_CH0_HANDLER (DMA_FIRST_HANDLER)
#define DMA_CH1_HANDLER (DMA_FIRST_HANDLER + 1)
#define DMA_CH2_HANDLER (DMA_FIRST_HANDLER + 2)
#define DMA_CH3_HANDLER (DMA_FIRST_HANDLER + 3)
#define DMA_CH4_HANDLER (DMA_FIRST_HANDLER + 4)
#define DMA_LAST_HANDLER DMA_CH4_HANDLER

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

// GDMA channel count
#define ESP32_GDMA_CHANNEL_COUNT 5

dmaIdentifier_e dmaGetFreeIdentifier(void);

// Initialise the GDMA hardware (bus clock, reset)
void esp32DmaInit(void);
