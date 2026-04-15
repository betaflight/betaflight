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
#include <string.h>

#include "platform.h"

#if ENABLE_CAN

#include "drivers/can/can.h"
#include "drivers/can/can_impl.h"
#include "drivers/io.h"

#include "pg/can.h"

canDevice_t canDevice[CANDEV_COUNT];

void canPinConfigure(const canPinConfig_t *pConfig)
{
    for (size_t hwindex = 0; hwindex < ARRAYLEN(canHardware); hwindex++) {
        const canHardware_t *hw = &canHardware[hwindex];

        if (!hw->reg) {
            continue;
        }

        const canDevice_e device = hw->device;
        canDevice_t *pDev = &canDevice[device];

        for (int pindex = 0; pindex < CAN_MAX_PIN_SEL; pindex++) {
            if (pConfig[device].ioTagTx && pConfig[device].ioTagTx == hw->txPins[pindex].pin) {
                pDev->tx = hw->txPins[pindex].pin;
                pDev->txAF = hw->txPins[pindex].af;
            }
            if (pConfig[device].ioTagRx && pConfig[device].ioTagRx == hw->rxPins[pindex].pin) {
                pDev->rx = hw->rxPins[pindex].pin;
                pDev->rxAF = hw->rxPins[pindex].af;
            }
        }

        if (pDev->tx && pDev->rx) {
            pDev->reg = hw->reg;
#if PLATFORM_TRAIT_RCC
            pDev->rcc = hw->rcc;
#endif
            pDev->irq0 = hw->irq0;
            pDev->irq1 = hw->irq1;
        }
    }
}

#endif // ENABLE_CAN
