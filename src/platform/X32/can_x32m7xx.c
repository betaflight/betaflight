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

#include "common/utils.h"
#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"
#include "platform/rcc.h"

// X32M7 FDCAN hardware table.
//
//  - 8 FDCAN instances: FDCAN1/2/5/6 on APB1, FDCAN3/4/7/8 on APB2.
//  - Each instance has its own RCC clock-enable bit (M7 core) and
//    kernel clock selector.
//  - Message RAM lives in SRAM5: Bank1 (0x30050000) for FDCAN1-4,
//    Bank2 (0x30054000) for FDCAN5-8.

const canHardware_t canHardware[CANDEV_COUNT] = {
    // ---- FDCAN1 (APB1)  — 5 pin groups ----
    {
        .device = CANDEV_1,
        .reg = (canResource_t *)FDCAN1,
        .txPins = {
            { DEFIO_TAG_E(PD1),  6  },   // AF6  FDCAN1_TX
            { DEFIO_TAG_E(PA12), 9  },   // AF9  FDCAN1_TX
            { DEFIO_TAG_E(PB9),  9  },   // AF9  FDCAN1_TX
            { DEFIO_TAG_E(PH13), 7  },   // AF7  FDCAN1_TX
            { DEFIO_TAG_E(PI8),  5  },   // AF5  FDCAN1_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PD0),  8  },   // AF8  FDCAN1_RX
            { DEFIO_TAG_E(PA11), 9  },   // AF9  FDCAN1_RX
            { DEFIO_TAG_E(PB8),  10 },   // AF10 FDCAN1_RX
            { DEFIO_TAG_E(PH14), 9  },   // AF9  FDCAN1_RX
            { DEFIO_TAG_E(PI9),  8  },   // AF8  FDCAN1_RX
        },
        .rcc = RCC_APB1_5(FDCAN1),
        .irq0 = FDCAN1_INT0_IRQn,
        .irq1 = FDCAN1_INT1_IRQn,
    },
    // ---- FDCAN2 (APB1)  — 3 pin groups ----
    {
        .device = CANDEV_2,
        .reg = (canResource_t *)FDCAN2,
        .txPins = {
            { DEFIO_TAG_E(PH9),  9  },   // AF9  FDCAN2_TX
            { DEFIO_TAG_E(PB13), 10 },   // AF10 FDCAN2_TX
            { DEFIO_TAG_E(PB6),  10 },   // AF10 FDCAN2_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PH8),  10 },   // AF10 FDCAN2_RX
            { DEFIO_TAG_E(PB12), 10 },   // AF10 FDCAN2_RX
            { DEFIO_TAG_E(PB5),  11 },   // AF11 FDCAN2_RX
        },
        .rcc = RCC_APB1_5(FDCAN2),
        .irq0 = FDCAN2_INT0_IRQn,
        .irq1 = FDCAN2_INT1_IRQn,
    },
    // ---- FDCAN3 (APB2)  — 4 pin groups ----
    {
        .device = CANDEV_3,
        .reg = (canResource_t *)FDCAN3,
        .txPins = {
            { DEFIO_TAG_E(PF7),  10 },   // AF10 FDCAN3_TX
            { DEFIO_TAG_E(PF15), 8  },   // AF8  FDCAN3_TX
            { DEFIO_TAG_E(PD13), 7  },   // AF7  FDCAN3_TX
            { DEFIO_TAG_E(PG9),  8  },   // AF8  FDCAN3_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PF6),  11 },   // AF11 FDCAN3_RX
            { DEFIO_TAG_E(PF14), 7  },   // AF7  FDCAN3_RX
            { DEFIO_TAG_E(PD12), 9  },   // AF9  FDCAN3_RX
            { DEFIO_TAG_E(PG10), 11 },   // AF11 FDCAN3_RX
        },
        .rcc = RCC_APB2_4(FDCAN3),
        .irq0 = FDCAN3_INT0_IRQn,
        .irq1 = FDCAN3_INT1_IRQn,
    },
    // ---- FDCAN4 (APB2)  — 3 pin groups ----
    {
        .device = CANDEV_4,
        .reg = (canResource_t *)FDCAN4,
        .txPins = {
            { DEFIO_TAG_E(PE2),  7  },   // AF7  FDCAN4_TX
            { DEFIO_TAG_E(PI10), 8  },   // AF8  FDCAN4_TX
            { DEFIO_TAG_E(PF0),  9  },   // AF9  FDCAN4_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PE3),  5  },   // AF5  FDCAN4_RX
            { DEFIO_TAG_E(PI11), 6  },   // AF6  FDCAN4_RX
            { DEFIO_TAG_E(PF1),  10 },   // AF10 FDCAN4_RX
        },
        .rcc = RCC_APB2_4(FDCAN4),
        .irq0 = FDCAN4_INT0_IRQn,
        .irq1 = FDCAN4_INT1_IRQn,
    },
    // ---- FDCAN5 (APB1)  — 3 pin groups ----
    {
        .device = CANDEV_5,
        .reg = (canResource_t *)FDCAN5,
        .txPins = {
            { DEFIO_TAG_E(PF4),  11 },   // AF11 FDCAN5_TX
            { DEFIO_TAG_E(PH4),  8  },   // AF8  FDCAN5_TX
            { DEFIO_TAG_E(PE8),  8  },   // AF8  FDCAN5_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PF5),  9  },   // AF9  FDCAN5_RX
            { DEFIO_TAG_E(PH5),  8  },   // AF8  FDCAN5_RX
            { DEFIO_TAG_E(PE9),  8  },   // AF8  FDCAN5_RX
        },
        .rcc = RCC_APB1_5(FDCAN5),
        .irq0 = FDCAN5_INT0_IRQn,
        .irq1 = FDCAN5_INT1_IRQn,
    },
    // ---- FDCAN6 (APB1)  — 3 pin groups ----
    {
        .device = CANDEV_6,
        .reg = (canResource_t *)FDCAN6,
        .txPins = {
            { DEFIO_TAG_E(PI12), 6  },   // AF6  FDCAN6_TX
            { DEFIO_TAG_E(PF12), 6  },   // AF6  FDCAN6_TX
            { DEFIO_TAG_E(PE13), 7  },   // AF7  FDCAN6_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PI13), 6  },   // AF6  FDCAN6_RX
            { DEFIO_TAG_E(PF13), 8  },   // AF8  FDCAN6_RX
            { DEFIO_TAG_E(PE14), 9  },   // AF9  FDCAN6_RX
        },
        .rcc = RCC_APB1_5(FDCAN6),
        .irq0 = FDCAN6_INT0_IRQn,
        .irq1 = FDCAN6_INT1_IRQn,
    },
    // ---- FDCAN7 (APB2)  — 4 pin groups ----
    {
        .device = CANDEV_7,
        .reg = (canResource_t *)FDCAN7,
        .txPins = {
            { DEFIO_TAG_E(PJ3),  8  },   // AF8  FDCAN7_TX
            { DEFIO_TAG_E(PH6),  10 },   // AF10 FDCAN7_TX
            { DEFIO_TAG_E(PJ6),  4  },   // AF4  FDCAN7_TX
            { DEFIO_TAG_E(PG7),  7  },   // AF7  FDCAN7_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PJ4),  7  },   // AF7  FDCAN7_RX
            { DEFIO_TAG_E(PH7),  8  },   // AF8  FDCAN7_RX
            { DEFIO_TAG_E(PJ7),  4  },   // AF4  FDCAN7_RX
            { DEFIO_TAG_E(PG8),  8  },   // AF8  FDCAN7_RX
        },
        .rcc = RCC_APB2_4(FDCAN7),
        .irq0 = FDCAN7_INT0_IRQn,
        .irq1 = FDCAN7_INT1_IRQn,
    },
    // ---- FDCAN8 (APB2)  — 3 pin groups ----
    {
        .device = CANDEV_8,
        .reg = (canResource_t *)FDCAN8,
        .txPins = {
            { DEFIO_TAG_E(PJ0),  7  },   // AF7  FDCAN8_TX
            { DEFIO_TAG_E(PJ8),  8  },   // AF8  FDCAN8_TX
            { DEFIO_TAG_E(PG2),  9  },   // AF9  FDCAN8_TX
        },
        .rxPins = {
            { DEFIO_TAG_E(PJ1),  8  },   // AF8  FDCAN8_RX
            { DEFIO_TAG_E(PJ9),  8  },   // AF8  FDCAN8_RX
            { DEFIO_TAG_E(PG3),  7  },   // AF7  FDCAN8_RX
        },
        .rcc = RCC_APB2_4(FDCAN8),
        .irq0 = FDCAN8_INT0_IRQn,
        .irq1 = FDCAN8_INT1_IRQn,
    },
};

#endif // ENABLE_CAN
