#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
	DEF_TIM( TIM2, CH4, PA3, TIM_USE_PWM | TIM_USE_PPM, TIMER_INPUT_ENABLED ) ,
    // Main outputs 6 PWM
	DEF_TIM( TIM4, CH1, PB6, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
	DEF_TIM( TIM4, CH2, PB7, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
	DEF_TIM( TIM8, CH2, PB8, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
	DEF_TIM( TIM8, CH3, PB9, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // PWM6 - PA0
    { TIM3,  IO_TAG(PC7),  TIM_Channel_2, TIM_USE_MOTOR, 1, GPIO_AF_2, NULL, 0 }, // PWM7 - PA1
//	DEF_TIM( TIM3, CH1, PC6, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
//	DEF_TIM( TIM3, CH2, PC7, TIM_USE_MOTOR,             TIMER_OUTPUT_ENABLED ),
    // Servo outputs 2 PWM
//	DEF_TIM( TIM3, CH1, PC6, TIM_USE_SERVO,             TIMER_OUTPUT_ENABLED ),
//	DEF_TIM( TIM3, CH2, PC7, TIM_USE_SERVO,             TIMER_OUTPUT_ENABLED ),
	// Additional outputs
	DEF_TIM( TIM16,CH1,  PA6, TIM_USE_TRANSPONDER,		TIMER_OUTPUT_ENABLED ),
	DEF_TIM( TIM1 ,CH1N, PA7, TIM_USE_LED,  			TIMER_OUTPUT_ENABLED ),
};
