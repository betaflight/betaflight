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

#define IOCFG_OUT_PP        0
#define IOCFG_OUT_OD        0
#define IOCFG_AF_PP         0
#define IOCFG_AF_OD         0
#define IOCFG_IPD           0
#define IOCFG_IPU           0
#define IOCFG_IN_FLOATING   0

#define SPIDEV_COUNT        0

// no serial pins are defined for the simulator
#define SERIAL_TRAIT_PIN_CONFIG 0

//basic DMA support
typedef enum dmaIdentifier_enum {
    DMA_NONE = 0,
    DMA1_CH1_HANDLER = 1,
    DMA1_CH2_HANDLER,
    DMA1_CH3_HANDLER,
    DMA1_CH4_HANDLER,
    DMA1_CH5_HANDLER,
    DMA1_CH6_HANDLER,
    DMA1_CH7_HANDLER,
    DMA_LAST_HANDLER = DMA1_CH7_HANDLER
} dmaIdentifier_e;


/// @todo [DMA-Codeclean] Check if this is correct, Simulator could be similar to F4
#define DMA_TRAIT_ARCH_CHANNEL_TYPE