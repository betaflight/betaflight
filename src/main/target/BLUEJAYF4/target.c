
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
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8), 
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),     
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),    
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn,       0, Mode_IPD,   GPIO_PinSource7, GPIO_AF_TIM8, 0 },             // PPM IN
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn,          1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_TIM5, 0 },             // S1_OUT
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn,          1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_TIM5, 0 },             // S2_OUT 
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn,          1, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_TIM2, 0 },             // S3_OUT
    { TIM9, GPIOA, Pin_3, TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_TIM9, 0 },             // S4_OUT
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn,          1, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_TIM3, 0 },             // S5_OUT
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn,          1, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_TIM3, 0 },             // S6_OUT
};

