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
#ifdef RX_PWM1_PIN
    pwmConfig->ioTags[0] = IO_TAG(RX_PWM1_PIN);
#endif
#ifdef RX_PWM2_PIN
    pwmConfig->ioTags[1] = IO_TAG(RX_PWM2_PIN);
#endif
#ifdef RX_PWM3_PIN
    pwmConfig->ioTags[2] = IO_TAG(RX_PWM3_PIN);
#endif
#ifdef RX_PWM4_PIN
    pwmConfig->ioTags[3] = IO_TAG(RX_PWM4_PIN);
#endif
}
#endif

#ifdef USE_RX_PPM
PG_REGISTER_WITH_RESET_FN(ppmConfig_t, ppmConfig, PG_PPM_CONFIG, 0);

void pgResetFn_ppmConfig(ppmConfig_t *ppmConfig)
{
#ifdef RX_PPM_PIN
    ppmConfig->ioTag = IO_TAG(RX_PPM_PIN);
#else
    ppmConfig->ioTag = IO_TAG_NONE;
#endif
}
#endif

#endif
