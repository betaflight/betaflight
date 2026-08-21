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

/*
 * HPMicro timer clock frequency helpers.
 */

#include "platform.h"

#ifdef USE_TIMER

#include "drivers/timer.h"
#include "timer_hpmicro.h"
#include "hpm_clock_drv.h"

FAST_CODE uint32_t getPWMFre(TIM_TypeDef *timHw)
{
    uint32_t fre = clock_get_frequency(timerRCC(timHw));
    return fre;
}

FAST_CODE uint32_t timerClock(const timerHardware_t *timHw)
{
    return getPWMFre((TIM_TypeDef *) timHw->tim);
}

#endif
