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

#include "platform.h"

#if ENABLE_CAN

#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"
#include "platform/rcc.h"

// H7 FDCAN specifics:
//  - All instances share a single RCC enable bit (RCC_APB1HENR_FDCANEN),
//    so RCC_APB1H(FDCAN) is stored on every row. The peripheral clock tree
//    on H7 sources FDCAN from the dedicated FDCAN kernel clock (sel'd via
//    RCC->D2CCIP1R.FDCANSEL) which is independent from the bus clock.
//  - Only STM32H72x/H73x variants include FDCAN3; on H74x/H75x it is absent
//    and the FDCAN3 row is skipped via #ifdef FDCAN3.
const canHardware_t canHardware[CANDEV_COUNT] = {
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)FDCAN1,
        .txPins = {
            { DEFIO_TAG_E(PA12), GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB9),  GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD1),  GPIO_AF9_FDCAN1 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA11), GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB8),  GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD0),  GPIO_AF9_FDCAN1 },
        },
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN1_IT0_IRQn,
        .irq1 = FDCAN1_IT1_IRQn,
    },
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)FDCAN2,
        .txPins = {
            { DEFIO_TAG_E(PB6),  GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB13), GPIO_AF9_FDCAN2 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PB5),  GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB12), GPIO_AF9_FDCAN2 },
        },
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN2_IT0_IRQn,
        .irq1 = FDCAN2_IT1_IRQn,
    },
#if defined(FDCAN3)
    {
        .device = CANDEV_3,
        .reg = (canResource_t *)FDCAN3,
        .txPins = {
            { DEFIO_TAG_E(PA15), GPIO_AF2_FDCAN3 },
            { DEFIO_TAG_E(PD13), GPIO_AF2_FDCAN3 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA8),  GPIO_AF2_FDCAN3 },
            { DEFIO_TAG_E(PD12), GPIO_AF2_FDCAN3 },
        },
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN3_IT0_IRQn,
        .irq1 = FDCAN3_IT1_IRQn,
    },
#else
    // Placeholder for H74x/H75x which lack FDCAN3. Empty reg field signals
    // "not available" to canPinConfigure / canInit.
    { 0 },
#endif
};

#endif // ENABLE_CAN
