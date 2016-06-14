
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),     // Swap to servo if needed
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1 or #3
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #5
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8),      // servo #8
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),      // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_6, 0}, // PA8 - AF6
    { TIM1,  IO_TAG(PA9),  TIM_Channel_2, TIM1_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_6, 0}, // PA9 - AF6
    { TIM1,  IO_TAG(PA10), TIM_Channel_3, TIM1_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_6, 0}, // PA10 - AF6
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, TIM3_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, 0}, // PB4 - AF2
    { TIM4,  IO_TAG(PB6),  TIM_Channel_1, TIM4_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, 0}, // PB6 - AF2 - not working yet
    { TIM4,  IO_TAG(PB7),  TIM_Channel_2, TIM4_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, 0}, // PB7 - AF2 - not working yet
    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, TIM4_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, 0}, // PB8 - AF2
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, TIM4_IRQn,               0, IOCFG_AF_PP_PD, GPIO_AF_2, 0}, // PB9 - AF2
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PA0 - untested
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PA1 - untested
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_9, 0}, // PA2 - untested
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_9, 0}, // PA3 - untested
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // PA6 - untested
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, IOCFG_AF_PP,    GPIO_AF_1, 0}  // PA7 - untested
};

