/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	
#ifdef FPVM_BETAFLIGHTF7
    DEF_TIM(TIM8, CH3, PC8, TIM_USE_ANY,   0, 0), // USED FOR CAMERA CONTROL
#endif
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_NONE,  0, 1 ), // RC1 / PPM, unusable

    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MOTOR, 0, 0 ), // M1
    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_MOTOR, 0, 0 ), // M2
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_MOTOR, 0, 2 ), // M3
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MOTOR, 0, 1 ), // M4

    DEF_TIM(TIM4, CH1, PD12, TIM_USE_LED,   0, 0 ), // LED

    // Backdoor timers
    DEF_TIM(TIM2, CH3, PB10, TIM_USE_NONE,  0, 0 ), // UART3_TX, I2C2_SCL
    DEF_TIM(TIM2, CH4, PB11, TIM_USE_NONE,  0, 0 ), // UART3_RX, I2C2_SDA
    DEF_TIM(TIM8, CH1, PC6,  TIM_USE_NONE,  0, 0 ), // UART6_TX
    DEF_TIM(TIM8, CH2, PC7,  TIM_USE_NONE,  0, 0 ), // UART6_RX

    // For ESC serial
    DEF_TIM(TIM9, CH1, PA2,  TIM_USE_NONE,  0, 0 ), // UART2_TX (unwired)

};
