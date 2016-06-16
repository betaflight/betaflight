
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    // No PPM
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    // No PPM
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {

    { TIM4,  IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0}, // PWM1 - PB6
    { TIM4,  IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0}, // PWM2 - PB6
    { TIM4,  IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0},  // PWM3 - PB8
    { TIM4,  IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0},  // PWM4 - PB9

    { TIM3,  IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0}, // PWM5 - PB0  - *TIM3_CH3
    { TIM3,  IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP, GPIO_AF_2, 0}, // PWM6 - PB1  - *TIM3_CH4

};


