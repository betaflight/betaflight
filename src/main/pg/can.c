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

#include "platform.h"

#if ENABLE_CAN

#include "drivers/io.h"

#include "pg/can.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#ifndef CAN1_TX_PIN
#define CAN1_TX_PIN NONE
#define CAN1_RX_PIN NONE
#endif

#ifndef CAN2_TX_PIN
#define CAN2_TX_PIN NONE
#define CAN2_RX_PIN NONE
#endif

#ifndef CAN3_TX_PIN
#define CAN3_TX_PIN NONE
#define CAN3_RX_PIN NONE
#endif

typedef struct canDefaultConfig_s {
    canDevice_e device;
    ioTag_t tx;
    ioTag_t rx;
} canDefaultConfig_t;

static const canDefaultConfig_t canDefaultConfig[] = {
    { CANDEV_1, IO_TAG(CAN1_TX_PIN), IO_TAG(CAN1_RX_PIN) },
    { CANDEV_2, IO_TAG(CAN2_TX_PIN), IO_TAG(CAN2_RX_PIN) },
    { CANDEV_3, IO_TAG(CAN3_TX_PIN), IO_TAG(CAN3_RX_PIN) },
};

PG_REGISTER_ARRAY_WITH_RESET_FN(canPinConfig_t, CANDEV_COUNT, canPinConfig, PG_CAN_PIN_CONFIG, 0);

void pgResetFn_canPinConfig(canPinConfig_t *config)
{
    for (size_t i = 0; i < ARRAYLEN(canDefaultConfig); i++) {
        const canDefaultConfig_t *defconf = &canDefaultConfig[i];
        config[defconf->device].ioTagTx = defconf->tx;
        config[defconf->device].ioTagRx = defconf->rx;
    }
}

#endif // ENABLE_CAN
