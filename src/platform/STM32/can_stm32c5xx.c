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
//    variant with FDCAN (C562, C593, C5A3) is selected.
//  - Silicon exposes at most FDCAN1 and FDCAN2. The CANDEV_3 row is emitted as
//    a zero-initialised placeholder so the fixed-length canHardware[] array
//    still has CANDEV_COUNT entries; canPinConfigure() skips rows whose reg
//    pointer is NULL.
//  - FDCAN clock-enable is on the APB1H bus (RCC_APB1HENR_FDCANEN), same
//    layout as H5/H7. The encoded RCC_APB1H(FDCAN) macro handles it.
//  - Pin AF values use the HAL2 naming (HAL_GPIO_AF9_FDCANx); the numeric
//    value is still 9 as on G4/H7, but the symbol name is HAL_-prefixed on
//    this SDK.
//  - The pin options are per *variant*, not per family, and the two differ in
//    a way that matters: C562 has one FDCAN, so pins that are FDCAN2 lines on
//    a C593 (PB5/PB6/PB12/PB13) are FDCAN1 lines there. Listing one set for
//    the family would hand those four pins to the wrong instance, so C562 has
//    its own row below.
//  - The C562 list is DS14927's complete AF9 set restricted to the ports
//    Betaflight compiles in for this part (A-E; C562 has no GPIOF and no
//    GPIOH, so the datasheet's PH2 FDCAN1_RX option is not reachable).
//  - The second row serves both C593 and C5A3, whose FDCAN AF maps are
//    identical: DS15136 (STM32C59xxx, covering C591 and C593) and DS15137
//    (STM32C5Axxx) list the same seven TX and six RX pins for FDCAN1 and the
//    same three each for FDCAN2. Every pin the original commit listed is in
//    that set; what is added is the rest. PH2 is left out for the same reason
//    as on C562 - Betaflight compiles no GPIOH for any C5 - so FDCAN1_RX on
//    PH2 is not reachable. No C593 target exists yet, so that half is read
//    from the datasheet rather than proved by a build; C5A3 does build.
const canHardware_t canHardware[CANDEV_COUNT] = {
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)FDCAN1,
#if defined(STM32C562xx)
        .txPins = {
            { DEFIO_TAG_E(PA12), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB6),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB7),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB9),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB13), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PC13), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD1),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD5),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PE1),  HAL_GPIO_AF9_FDCAN1 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA11), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB5),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB8),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB12), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PC14), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD0),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PE0),  HAL_GPIO_AF9_FDCAN1 },
        },
#else
        .txPins = {
            { DEFIO_TAG_E(PA12), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB7),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB9),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PC13), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD1),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD5),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PE1),  HAL_GPIO_AF9_FDCAN1 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PA11), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PB8),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PC14), HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PD0),  HAL_GPIO_AF9_FDCAN1 },
            { DEFIO_TAG_E(PE0),  HAL_GPIO_AF9_FDCAN1 },
        },
#endif
        .rcc = RCC_APB1H(FDCAN),
        .irq0 = FDCAN1_IT0_IRQn,
        .irq1 = FDCAN1_IT1_IRQn,
    },
#if defined(FDCAN2)
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)FDCAN2,
        .txPins = {
            { DEFIO_TAG_E(PA10), HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB6),  HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB13), HAL_GPIO_AF9_FDCAN2 },
        },
        .rxPins = {
            { DEFIO_TAG_E(PB5),  HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PB12), HAL_GPIO_AF9_FDCAN2 },
            { DEFIO_TAG_E(PD9),  HAL_GPIO_AF9_FDCAN2 },
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
