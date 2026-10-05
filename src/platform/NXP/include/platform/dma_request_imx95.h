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

// eDMA request ids of the i.MX95 peripherals: the eDMA number (2 or 3) in the
// high byte and the request number from fsl_edma_soc.h in the low byte.
#define NXP_DMA_REQ(edma, request)       (((edma) << 8) | (request))
#define NXP_DMA_REQ_EDMA(id)             ((id) >> 8)
#define NXP_DMA_REQ_SOURCE(id)           ((id) & 0xff)

#define NXP_DMA_REQ_LPUART1_TX           NXP_DMA_REQ(3, 20)
#define NXP_DMA_REQ_LPUART1_RX           NXP_DMA_REQ(3, 21)
#define NXP_DMA_REQ_LPUART2_TX           NXP_DMA_REQ(3, 22)
#define NXP_DMA_REQ_LPUART2_RX           NXP_DMA_REQ(3, 23)
#define NXP_DMA_REQ_LPUART3_TX           NXP_DMA_REQ(2, 17)
#define NXP_DMA_REQ_LPUART3_RX           NXP_DMA_REQ(2, 18)
#define NXP_DMA_REQ_LPUART4_TX           NXP_DMA_REQ(2, 19)
#define NXP_DMA_REQ_LPUART4_RX           NXP_DMA_REQ(2, 20)
#define NXP_DMA_REQ_LPUART5_TX           NXP_DMA_REQ(2, 21)
#define NXP_DMA_REQ_LPUART5_RX           NXP_DMA_REQ(2, 22)
#define NXP_DMA_REQ_LPUART6_TX           NXP_DMA_REQ(2, 23)
#define NXP_DMA_REQ_LPUART6_RX           NXP_DMA_REQ(2, 24)
#define NXP_DMA_REQ_LPUART7_TX           NXP_DMA_REQ(2, 87)
#define NXP_DMA_REQ_LPUART7_RX           NXP_DMA_REQ(2, 88)
#define NXP_DMA_REQ_LPUART8_TX           NXP_DMA_REQ(2, 89)
#define NXP_DMA_REQ_LPUART8_RX           NXP_DMA_REQ(2, 90)

#define NXP_DMA_REQ_LPSPI1_TX            NXP_DMA_REQ(3, 16)
#define NXP_DMA_REQ_LPSPI1_RX            NXP_DMA_REQ(3, 17)
#define NXP_DMA_REQ_LPSPI2_TX            NXP_DMA_REQ(3, 18)
#define NXP_DMA_REQ_LPSPI2_RX            NXP_DMA_REQ(3, 19)
#define NXP_DMA_REQ_LPSPI3_TX            NXP_DMA_REQ(2, 12)
#define NXP_DMA_REQ_LPSPI3_RX            NXP_DMA_REQ(2, 13)
#define NXP_DMA_REQ_LPSPI4_TX            NXP_DMA_REQ(2, 14)
#define NXP_DMA_REQ_LPSPI4_RX            NXP_DMA_REQ(2, 15)
#define NXP_DMA_REQ_LPSPI5_TX            NXP_DMA_REQ(2, 79)
#define NXP_DMA_REQ_LPSPI5_RX            NXP_DMA_REQ(2, 80)
#define NXP_DMA_REQ_LPSPI6_TX            NXP_DMA_REQ(2, 81)
#define NXP_DMA_REQ_LPSPI6_RX            NXP_DMA_REQ(2, 82)
#define NXP_DMA_REQ_LPSPI7_TX            NXP_DMA_REQ(2, 83)
#define NXP_DMA_REQ_LPSPI7_RX            NXP_DMA_REQ(2, 84)
#define NXP_DMA_REQ_LPSPI8_TX            NXP_DMA_REQ(2, 85)
#define NXP_DMA_REQ_LPSPI8_RX            NXP_DMA_REQ(2, 86)

#define NXP_DMA_REQ_LPI2C1_TX            NXP_DMA_REQ(3, 12)
#define NXP_DMA_REQ_LPI2C1_RX            NXP_DMA_REQ(3, 13)
#define NXP_DMA_REQ_LPI2C2_TX            NXP_DMA_REQ(3, 14)
#define NXP_DMA_REQ_LPI2C2_RX            NXP_DMA_REQ(3, 15)
#define NXP_DMA_REQ_LPI2C3_TX            NXP_DMA_REQ(2, 8)
#define NXP_DMA_REQ_LPI2C3_RX            NXP_DMA_REQ(2, 9)
#define NXP_DMA_REQ_LPI2C4_TX            NXP_DMA_REQ(2, 10)
#define NXP_DMA_REQ_LPI2C4_RX            NXP_DMA_REQ(2, 11)
#define NXP_DMA_REQ_LPI2C5_TX            NXP_DMA_REQ(2, 71)
#define NXP_DMA_REQ_LPI2C5_RX            NXP_DMA_REQ(2, 72)
#define NXP_DMA_REQ_LPI2C6_TX            NXP_DMA_REQ(2, 73)
#define NXP_DMA_REQ_LPI2C6_RX            NXP_DMA_REQ(2, 74)
#define NXP_DMA_REQ_LPI2C7_TX            NXP_DMA_REQ(2, 75)
#define NXP_DMA_REQ_LPI2C7_RX            NXP_DMA_REQ(2, 76)
#define NXP_DMA_REQ_LPI2C8_TX            NXP_DMA_REQ(2, 77)
#define NXP_DMA_REQ_LPI2C8_RX            NXP_DMA_REQ(2, 78)

#define NXP_DMA_REQ_FLEXCAN1             NXP_DMA_REQ(3, 1)
#define NXP_DMA_REQ_FLEXCAN2             NXP_DMA_REQ(2, 1)
#define NXP_DMA_REQ_FLEXCAN3             NXP_DMA_REQ(2, 57)
#define NXP_DMA_REQ_FLEXCAN4             NXP_DMA_REQ(2, 91)
#define NXP_DMA_REQ_FLEXCAN5             NXP_DMA_REQ(2, 92)

#define NXP_DMA_REQ_FLEXIO1_SHIFTER0     NXP_DMA_REQ(2, 37)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER1     NXP_DMA_REQ(2, 38)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER2     NXP_DMA_REQ(2, 39)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER3     NXP_DMA_REQ(2, 40)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER4     NXP_DMA_REQ(2, 41)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER5     NXP_DMA_REQ(2, 42)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER6     NXP_DMA_REQ(2, 43)
#define NXP_DMA_REQ_FLEXIO1_SHIFTER7     NXP_DMA_REQ(2, 44)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER0     NXP_DMA_REQ(2, 45)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER1     NXP_DMA_REQ(2, 46)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER2     NXP_DMA_REQ(2, 47)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER3     NXP_DMA_REQ(2, 48)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER4     NXP_DMA_REQ(2, 49)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER5     NXP_DMA_REQ(2, 50)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER6     NXP_DMA_REQ(2, 51)
#define NXP_DMA_REQ_FLEXIO2_SHIFTER7     NXP_DMA_REQ(2, 52)
