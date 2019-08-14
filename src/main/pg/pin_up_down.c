/*
 * This file is part of Betaflight.
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

#ifdef USE_PINIO

#include "pg/pg_ids.h"
#include "pin_up_down.h"
#include "drivers/io.h"

PG_REGISTER_WITH_RESET_TEMPLATE(pinUpDownConfig_t, pinPullupConfig, PG_PULLUP_CONFIG, 0);
PG_REGISTER_WITH_RESET_TEMPLATE(pinUpDownConfig_t, pinPulldownConfig, PG_PULLDOWN_CONFIG, 0);

PG_RESET_TEMPLATE(pinUpDownConfig_t, pinPullupConfig,
    .ioTag = {
        IO_TAG(NONE),
        IO_TAG(NONE),
    }
);

PG_RESET_TEMPLATE(pinUpDownConfig_t, pinPulldownConfig,
    .ioTag = {
        IO_TAG(NONE),
        IO_TAG(NONE),
    }
);
#endif
