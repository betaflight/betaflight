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

#ifdef USE_PIN_PULL_UP_DOWN

#include "drivers/io.h"
#include "pg/pg_ids.h"
#include "pin_pull_up_down.h"

static void resetPullUpDownConfig(pinPullUpDownConfig_t* config)
{
    for (uint8_t i = 0; i < PIN_PULL_UP_DOWN_COUNT; i++) {
        config[i].ioTag = IO_TAG(NONE);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(pinPullUpDownConfig_t, PIN_PULL_UP_DOWN_COUNT, pinPullupConfig, PG_PULLUP_CONFIG, 0);

void pgResetFn_pinPullupConfig(pinPullUpDownConfig_t *config)
{
    resetPullUpDownConfig(config);
}

PG_REGISTER_ARRAY_WITH_RESET_FN(pinPullUpDownConfig_t, PIN_PULL_UP_DOWN_COUNT, pinPulldownConfig, PG_PULLDOWN_CONFIG, 0);

void pgResetFn_pinPulldownConfig(pinPullUpDownConfig_t *config)
{
    resetPullUpDownConfig(config);
}
#endif
