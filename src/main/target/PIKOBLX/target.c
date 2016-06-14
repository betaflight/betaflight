
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM9  | (MAP_TO_PPM_INPUT << 8), // PPM input

    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
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
    { TIM3,  GPIOA, Pin_4,  TIM_Channel_2, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2}, // PWM1  - PA4  - *TIM3_CH2
    { TIM3,  GPIOA, Pin_6,  TIM_Channel_1, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_2}, // PWM2  - PA6  - *TIM3_CH1, TIM8_BKIN, TIM1_BKIN, TIM16_CH1
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2}, // PWM3  - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2}, // PWM4  - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1}, // PWM5  - PA1  - *TIM2_CH2, TIM15_CH1N
    { TIM2,  GPIOA, Pin_2,  TIM_Channel_3, TIM2_IRQn,               1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_1}, // PWM6  - PA2  - *TIM2_CH3, !TIM15_CH1
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9}, // PWM7  - PA3  - *TIM15_CH2, TIM2_CH4
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6}, // PWM8  - PA8  - *TIM1_CH1, TIM4_ETR

    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 0, Mode_AF_PP_PD, GPIO_PinSource7,  GPIO_AF_1}, // PPM   - PA7  - *TIM17_CH1, TIM1_CH1N, TIM8_CH1
};

