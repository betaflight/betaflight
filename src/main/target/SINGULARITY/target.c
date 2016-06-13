
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM9  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  GPIOA, Pin_15, TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource15, GPIO_AF_1, 0}, // PPM/SERIAL RX

    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2, 0}, // PWM1
    { TIM3,  GPIOB, Pin_5,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource5,  GPIO_AF_2, 0}, // PWM2
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2, 0}, // PWM3
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2, 0}, // PWM4
    { TIM16, GPIOB, Pin_8,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_1, 0}, // PWM5
    { TIM17, GPIOB, Pin_9,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource9,  GPIO_AF_1, 0}, // PWM6

    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_9, 0}, // SOFTSERIAL1 RX (NC)
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9, 0}, // SOFTSERIAL1 TX

    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6, 0}, // LED_STRIP
};

