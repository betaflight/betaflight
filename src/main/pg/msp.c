/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "msp.h"

PG_REGISTER_WITH_RESET_FN(mspConfig_t, mspConfig, PG_MSP_CONFIG, 0);

void pgResetFn_mspConfig(mspConfig_t *mspConfig)
{
    mspConfig->halfDuplex = 0;
    for (unsigned i = 0; i < MAX_MSP_PORT_COUNT; i++) {
        mspConfig->msp_uart[i] = SERIAL_PORT_NONE;
    }
}
