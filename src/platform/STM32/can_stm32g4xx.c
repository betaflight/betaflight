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

// On G4 all FDCAN instances share a single RCC enable bit
// (RCC_APB1ENR1_FDCANEN). RCC_ClockCmd() is idempotent when the same
// peripheral is enabled multiple times, so storing the same tag on
// each row is fine.
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
        .rcc = RCC_APB11(FDCAN),
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
        .rcc = RCC_APB11(FDCAN),
        .irq0 = FDCAN2_IT0_IRQn,
        .irq1 = FDCAN2_IT1_IRQn,
    },
    {
        .device = CANDEV_3,
        .reg = (canResource_t *)FDCAN3,
        .txPins = {
            { DEFIO_TAG_E(PA15), GPIO_AF11_FDCAN3 },
            { DEFIO_TAG_E(PB4),  GPIO_AF11_FDCAN3 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA8),  GPIO_AF11_FDCAN3 },
            { DEFIO_TAG_E(PB3),  GPIO_AF11_FDCAN3 },
        },
        .rcc = RCC_APB11(FDCAN),
        .irq0 = FDCAN3_IT0_IRQn,
        .irq1 = FDCAN3_IT1_IRQn,
    },
};

#endif // ENABLE_CAN
