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

#include "drivers/can/can.h"
#include "drivers/can/can_types.h"
#include "drivers/io_types.h"

#if PLATFORM_TRAIT_RCC
#include "platform/rcc_types.h"
#endif

// Maximum number of alternate pin options per TX/RX line.
#define CAN_MAX_PIN_SEL 3

typedef struct canPinDef_s {
    ioTag_t pin;
    uint8_t af;
} canPinDef_t;

typedef struct canHardware_s {
    canDevice_e device;
    canResource_t *reg;
    canPinDef_t txPins[CAN_MAX_PIN_SEL];
    canPinDef_t rxPins[CAN_MAX_PIN_SEL];
#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rcc;
#endif
    uint8_t irq0;                   // FDCANx_IT0_IRQn
    uint8_t irq1;                   // FDCANx_IT1_IRQn
} canHardware_t;

extern const canHardware_t canHardware[CANDEV_COUNT];

typedef struct canDevice_s {
    canResource_t *reg;
    ioTag_t tx;
    ioTag_t rx;
    uint8_t txAF;
    uint8_t rxAF;
#if PLATFORM_TRAIT_RCC
    rccPeriphTag_t rcc;
#endif
    uint8_t irq0;
    uint8_t irq1;
    canRxCallbackPtr rxCallback;
    bool initialized;
} canDevice_t;

extern canDevice_t canDevice[CANDEV_COUNT];

// Internal: shared init routine implemented per-platform in can_hw.c.
void canInitDevice(canDevice_e device);

// Internal: ISR dispatch helper invoked from named FDCANx IRQ handlers.
void canIrqHandler(canDevice_e device);

// Configure the device pin assignments from the PG config. Called from fc/init.c
// before canInit().
struct canPinConfig_s;
void canPinConfigure(const struct canPinConfig_s *pConfig);
