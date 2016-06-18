
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
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
    PWM1  | (MAP_TO_PPM_INPUT << 8),      // PPM input
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
    { TIM8, IO_TAG(PC7), TIM_Channel_2, TIM8_CC_IRQn,       0, IOCFG_IPD,    GPIO_AF_TIM8, 0 },             // PPM IN
    { TIM5, IO_TAG(PA0), TIM_Channel_1, TIM5_IRQn,          1, IOCFG_AF_PP,  GPIO_AF_TIM5, 0},             // S1_OUT
    { TIM5, IO_TAG(PA1), TIM_Channel_2, TIM5_IRQn,          1, IOCFG_AF_PP,  GPIO_AF_TIM5, 0},             // S2_OUT 
    { TIM2, IO_TAG(PA2), TIM_Channel_3, TIM2_IRQn,          1, IOCFG_AF_PP,  GPIO_AF_TIM2, 0},             // S3_OUT
    { TIM9, IO_TAG(PA3), TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 1, IOCFG_AF_PP,  GPIO_AF_TIM9, 0},             // S4_OUT
    { TIM3, IO_TAG(PB1), TIM_Channel_4, TIM3_IRQn,          1, IOCFG_AF_PP,  GPIO_AF_TIM3, 0},             // S5_OUT
    { TIM3, IO_TAG(PB0), TIM_Channel_3, TIM3_IRQn,          1, IOCFG_AF_PP,  GPIO_AF_TIM3, 0},             // S6_OUT
};

