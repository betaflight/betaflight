
#include <stdbool.h>
#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),         // PPM input
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #2 or servo #2 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #3 or #1
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #4 or #2
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #5 or #3
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #6 or #4
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #7 or #5
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #8 or #6
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),         // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),         // input #2
    PWM3  | (MAP_TO_PWM_INPUT << 8),         // input #3
    PWM4  | (MAP_TO_PWM_INPUT << 8),         // input #4
    PWM5  | (MAP_TO_PWM_INPUT << 8),         // input #5
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #2 or servo #2 (swap to servo if needed)
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #3 or #1
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #4 or #2
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #5 or #3
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #6 or #4
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #7 or #5
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #8 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),        // PPM input
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),     // motor #1
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),     // motor #2
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #1
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #2
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),     // servo #3
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8),     // servo #4
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8),     // servo #5
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),     // servo #6
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #7
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #8
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #9
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),     // servo #10
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),         // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),         // input #2
    PWM3  | (MAP_TO_PWM_INPUT << 8),         // input #3
    PWM4  | (MAP_TO_PWM_INPUT << 8),         // input #4
    PWM5  | (MAP_TO_PWM_INPUT << 8),         // input #5
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #1
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // motor #2
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),      // servo #1
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),      // servo #2
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),      // servo #3
    PWM11 | (MAP_TO_SERVO_OUTPUT << 8),      // servo #4
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8),      // servo #5
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),      // servo #6
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1, GPIOA, Pin_8, TIM_Channel_1, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM1},            // PWM1  - PA8  RC1
    { TIM1, GPIOB, Pin_0, TIM_Channel_2, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM1},            // PWM2  - PB0  RC2
    { TIM1, GPIOB, Pin_1, TIM_Channel_3, TIM1_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM1},            // PWM3  - PB1  RC3
    { TIM8, GPIOB, Pin_14, TIM_Channel_2, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource14, GPIO_AF_TIM8},          // PWM4  - PA14 RC4
    { TIM8, GPIOB, Pin_15, TIM_Channel_3, TIM8_CC_IRQn, 0, GPIO_Mode_AF, GPIO_PinSource15, GPIO_AF_TIM8},          // PWM5  - PA15 RC5

    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM4},               // PWM6  - PB8  OUT1
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM4},               // PWM7  - PB9  OUT2
    { TIM5, GPIOA, Pin_0, TIM_Channel_1, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource0, GPIO_AF_TIM5},               // PWM8  - PA0  OUT3
    { TIM5, GPIOA, Pin_1, TIM_Channel_2, TIM5_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource1, GPIO_AF_TIM5},               // PWM9  - PA1  OUT4
    { TIM3, GPIOC, Pin_6, TIM_Channel_1, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource6, GPIO_AF_TIM3},               // PWM10 - PC6  OUT5
    { TIM3, GPIOC, Pin_7, TIM_Channel_2, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource7, GPIO_AF_TIM3},               // PWM11 - PC7  OUT6
    { TIM3, GPIOC, Pin_8, TIM_Channel_3, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource8, GPIO_AF_TIM3},               // PWM13 - PC8  OUT7
    { TIM3, GPIOC, Pin_9, TIM_Channel_4, TIM3_IRQn, 1, GPIO_Mode_AF, GPIO_PinSource9, GPIO_AF_TIM3},               // PWM13 - PC9  OUT8
};

