/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <platform.h>

#if defined(USE_PWM) || defined(USE_PPM)

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/timer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx_pwm.h"

#ifdef USE_PWM
PG_REGISTER_WITH_RESET_FN(pwmConfig_t, pwmConfig, PG_PWM_CONFIG, 0);

void pgResetFn_pwmConfig(pwmConfig_t *pwmConfig)
{
    pwmConfig->inputFilteringMode = INPUT_FILTERING_DISABLED;
    int inputIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && inputIndex < PWM_INPUT_PORT_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PWM) {
            pwmConfig->ioTags[inputIndex] = timerHardware[i].tag;
            inputIndex++;
        }
    }
}
#endif

#ifdef USE_PPM
PG_REGISTER_WITH_RESET_FN(ppmConfig_t, ppmConfig, PG_PPM_CONFIG, 0);

void pgResetFn_ppmConfig(ppmConfig_t *ppmConfig)
{
#ifdef PPM_PIN
    ppmConfig->ioTag = IO_TAG(PPM_PIN);
#else
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_PPM) {
            ppmConfig->ioTag = timerHardware[i].tag;
            return;
        }
    }

    ppmConfig->ioTag = IO_TAG_NONE;
#endif
}
#endif

#endif
