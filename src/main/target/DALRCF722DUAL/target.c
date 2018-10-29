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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM4, CH2, PB7,  TIM_USE_PPM,   0, 0), // PPM&SBUS  

    DEF_TIM(TIM8, CH1, PC6,  TIM_USE_MOTOR, 0, 0),   // S1 (2,5)
    DEF_TIM(TIM8, CH2, PC7,  TIM_USE_MOTOR, 0, 0),   // S2 (2,5)
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_MOTOR, 0, 0),    // S3 (2,5)
    DEF_TIM(TIM8, CH4, PC9,  TIM_USE_MOTOR, 0, 0),   // S4 (2,5)  
    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_MOTOR, 0, 0),   // S5 (2,1) 
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_MOTOR, 0, 0),   // S6 (2,1)


    DEF_TIM(TIM2, CH1, PA15,  TIM_USE_LED, 0, 0),    // LED STRIP(1,5)

    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_CAMERA_CONTROL, 0, 0),     // FC CAM	
	
};
