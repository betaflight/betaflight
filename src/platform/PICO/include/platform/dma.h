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
    DMA_NONE = 0,
    DMA_CH1_HANDLER = 1,
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
    DMA_CH12_HANDLER,
#ifdef RP2350
    DMA_CH13_HANDLER,
    DMA_CH14_HANDLER,
    DMA_CH15_HANDLER,
    DMA_CH16_HANDLER,
    DMA_LAST_HANDLER = DMA_CH16_HANDLER
#else
    DMA_LAST_HANDLER = DMA_CH12_HANDLER
#endif
} dmaIdentifier_e;
