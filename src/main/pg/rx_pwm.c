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

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/timer.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx_pwm.h"

#ifdef USE_RX_PWM
PG_REGISTER_WITH_RESET_FN(pwmConfig_t, pwmConfig, PG_PWM_CONFIG, 0);

void pgResetFn_pwmConfig(pwmConfig_t *pwmConfig)
{
    pwmConfig->inputFilteringMode = INPUT_FILTERING_DISABLED;
    for (unsigned inputIndex = 0; inputIndex < PWM_INPUT_PORT_COUNT; inputIndex++) {
        pwmConfig->ioTags[inputIndex] = timerioTagGetByUsage(TIM_USE_PWM, inputIndex);
    }
}
#endif

#ifdef USE_RX_PPM
PG_REGISTER_WITH_RESET_FN(ppmConfig_t, ppmConfig, PG_PPM_CONFIG, 0);

void pgResetFn_ppmConfig(ppmConfig_t *ppmConfig)
{
    ppmConfig->ioTag = timerioTagGetByUsage(TIM_USE_PPM, 0);
}
#endif

#endif
