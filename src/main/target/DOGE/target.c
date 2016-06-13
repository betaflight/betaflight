
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
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
    // prevent crashing, but do nothing
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
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
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

const uint16_t airPWM[] = {
    // prevent crashing, but do nothing
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

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            0, Mode_AF_PP_PD,   GPIO_PinSource8,  GPIO_AF_6, 0}, // PWM1 - PA8

    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               1, Mode_AF_PP,      GPIO_PinSource8,  GPIO_AF_2, 0}, // PWM2 - PB8
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               1, Mode_AF_PP,      GPIO_PinSource9,  GPIO_AF_2, 0}, // PWM3 - PB9
    
    { TIM2,  GPIOA, Pin_10, TIM_Channel_4, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource10, GPIO_AF_10, 0}, // PMW4 - PA10
    { TIM2,  GPIOA, Pin_9,  TIM_Channel_3, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource9,  GPIO_AF_10, 0}, // PWM5 - PA9
    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource0,  GPIO_AF_1, 0}, // PWM6 - PA0
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP,      GPIO_PinSource1,  GPIO_AF_1, 0}, // PWM7 - PA1

    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP_PD,   GPIO_PinSource0,  GPIO_AF_2, 0}, // PWM8 - PB1
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP_PD,   GPIO_PinSource1,  GPIO_AF_2, 0}, // PWM9 - PB0
};

