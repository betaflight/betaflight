
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM8},          // PPM_IN

    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource3, GPIO_AF_TIM9},    // S1_OUT
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM3},             // S2_OUT
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM3},             // S3_OUT
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource2, GPIO_AF_TIM2},             // S4_OUT

//  { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},    // LED Strip

};

