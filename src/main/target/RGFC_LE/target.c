
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};

const uint16_t airPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM11 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM12 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
	0xFFFF
};

const uint16_t airPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM11 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM12 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
	0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn,    0, IOCFG_AF_PP_PD, GPIO_AF_1 },
    // Main outputs 8 PWM
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM8, IO_TAG(PC6), TIM_Channel_1, TIM8_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_4 },
    { TIM8, IO_TAG(PC7), TIM_Channel_2, TIM8_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_4 },
    { TIM8, IO_TAG(PC8), TIM_Channel_3, TIM8_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_4 },
    { TIM8, IO_TAG(PC9), TIM_Channel_4, TIM8_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_4 },
    // Servo outputs 4 PWM
    { TIM3, IO_TAG(PC6), TIM_Channel_1, TIM3_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC7), TIM_Channel_2, TIM3_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC8), TIM_Channel_3, TIM3_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
    { TIM3, IO_TAG(PC9), TIM_Channel_4, TIM3_IRQn,    1, IOCFG_AF_PP, GPIO_AF_2 },
	// Additional outputs
    { TIM16, IO_TAG(PA6), TIM_Channel_1, TIM1_UP_TIM16_IRQn,  1, IOCFG_AF_PP, GPIO_AF_1 },
    { TIM17, IO_TAG(PA7), TIM_Channel_2, TIM1_TRG_COM_TIM17_IRQn,  1, IOCFG_AF_PP, GPIO_AF_1 },
};

