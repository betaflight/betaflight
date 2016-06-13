
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM6  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM1  | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),     // input #6
    PWM7  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM8  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM9  | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM10 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #3
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};


const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM12, GPIOB, Pin_14, TIM_Channel_1, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM12, 0}, // PPM (5th pin on FlexiIO port)
    { TIM12, GPIOB, Pin_15, TIM_Channel_2, TIM8_BRK_TIM12_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM12, 0}, // S2_IN - GPIO_PartialRemap_TIM3
    { TIM8,  GPIOC, Pin_6,  TIM_Channel_1, TIM8_CC_IRQn,        0, GPIO_Mode_AF, GPIO_PinSource6,  GPIO_AF_TIM8, 0}, // S3_IN
    { TIM8,  GPIOC, Pin_7,  TIM_Channel_2, TIM8_CC_IRQn,        0, GPIO_Mode_AF, GPIO_PinSource7,  GPIO_AF_TIM8, 0}, // S4_IN
    { TIM8,  GPIOC, Pin_8,  TIM_Channel_3, TIM8_CC_IRQn,        0, GPIO_Mode_AF, GPIO_PinSource8,  GPIO_AF_TIM8, 0}, // S5_IN
    { TIM8,  GPIOC, Pin_9,  TIM_Channel_4, TIM8_CC_IRQn,        0, GPIO_Mode_AF, GPIO_PinSource9,  GPIO_AF_TIM8, 0}, // S6_IN

    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,           1, GPIO_Mode_AF, GPIO_PinSource0,  GPIO_AF_TIM3, 0}, // S1_OUT
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,           1, GPIO_Mode_AF, GPIO_PinSource1,  GPIO_AF_TIM3, 0}, // S2_OUT
    { TIM9,  GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM9_IRQn,  1, GPIO_Mode_AF, GPIO_PinSource3,  GPIO_AF_TIM9, 0}, // S3_OUT
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,           1, GPIO_Mode_AF, GPIO_PinSource2,  GPIO_AF_TIM2, 0}, // S4_OUT
    { TIM5,  GPIOA, Pin_1,  TIM_Channel_2, TIM5_IRQn,           1, GPIO_Mode_AF, GPIO_PinSource1,  GPIO_AF_TIM5, 0}, // S5_OUT - GPIO_PartialRemap_TIM3
    { TIM5,  GPIOA, Pin_0,  TIM_Channel_1, TIM5_IRQn,           1, GPIO_Mode_AF, GPIO_PinSource0,  GPIO_AF_TIM5, 0}, // S6_OUT
};


