
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM7  | (MAP_TO_PPM_INPUT << 8),
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_PWM_INPUT << 8),
    PWM10  | (MAP_TO_PWM_INPUT << 8),
    PWM11  | (MAP_TO_PWM_INPUT << 8),
    PWM12  | (MAP_TO_PWM_INPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    // TODO
    0xFFFF
};

const uint16_t airPWM[] = {
    // TODO
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1,  TIM1_CC_IRQn,           1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, PWM_INVERTED},
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, PWM_INVERTED},
    { TIM15, GPIOB, Pin_14, TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_1, PWM_INVERTED},
    { TIM15, GPIOB, Pin_15, TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1, PWM_INVERTED},
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_1, PWM_INVERTED},
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1, PWM_INVERTED},

    { TIM2,  GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_1, 0},
    { TIM2,  GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1, 0},
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1, 0},
    { TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1, 0},
    { TIM4,  GPIOA, Pin_13, TIM_Channel_2, TIM4_IRQn,               0, Mode_AF_PP, GPIO_PinSource13, GPIO_AF_10,0},
    { TIM8,  GPIOA, Pin_14, TIM_Channel_3, TIM8_CC_IRQn,            0, Mode_AF_PP, GPIO_PinSource14, GPIO_AF_5, 0},
};

