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

// C5 FDCAN specifics:
//  - C591 has no FDCAN peripheral; hardware-table rows only exist when a
//    variant with FDCAN (e.g. C593) is selected.
//  - Silicon exposes only FDCAN1 and FDCAN2. The CANDEV_3 row is emitted as
//    a zero-initialised placeholder so the fixed-length canHardware[] array
//    still has CANDEV_COUNT entries; canPinConfigure() skips rows whose reg
//    pointer is NULL.
//  - FDCAN clock-enable is on the APB1H bus (RCC_APB1HENR_FDCANEN), same
//    layout as H5/H7. The encoded RCC_APB1H(FDCAN) macro handles it.
//  - Pin AF values use the HAL2 naming (HAL_GPIO_AF9_FDCANx); the numeric
//    value is still 9 as on G4/H7, but the symbol name is HAL_-prefixed on
//    this SDK.
//  - Pin options listed here are the silicon-advertised FDCAN TX/RX lines
//    for C593 (AF9). An actual C593 target will be added in a separate PR;
//    the pin list can be extended then as needed.
const canHardware_t canHardware[CANDEV_COUNT] = {
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)FDCAN1,
        .txPins = {
            { DEFIO_TAG_E(PA12), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB9),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD1),  HAL_GPIO_AF9_FDCAN1 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA11), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB8),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD0),  HAL_GPIO_AF9_FDCAN1 },
        },
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN1_IT0_IRQn,
        .irq1 = FDCAN1_IT1_IRQn,
    },
#if defined(FDCAN2)
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)FDCAN2,
        .txPins = {
            { DEFIO_TAG_E(PB6),  HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB13), HAL_GPIO_AF9_FDCAN2 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PB5),  HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB12), HAL_GPIO_AF9_FDCAN2 },
        },
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN2_IT0_IRQn,
        .irq1 = FDCAN2_IT1_IRQn,
    },
#else
    { 0 },
#endif
    // C5 does not expose FDCAN3; leave CANDEV_3 row empty so the array
    // index layout stays consistent with G4 / H7.
    { 0 },
};

#endif // ENABLE_CAN
