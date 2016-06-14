
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #10
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
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), // server #6
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  GPIOA, Pin_0,  TIM_Channel_1, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_1,  0},  // RC_CH1 - PA0  - *TIM2_CH1
    { TIM2,  GPIOA, Pin_1,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_1,  0},  // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    { TIM2,  GPIOB, Pin_11, TIM_Channel_4, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_1,  0},  // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)
    { TIM2,  GPIOB, Pin_10, TIM_Channel_3, TIM2_IRQn,               0, Mode_AF_PP, GPIO_PinSource10, GPIO_AF_1,  0},  // RC_CH4 - PB10 - *TIM2_CH3, USART3_TX (AF7)
    { TIM3,  GPIOB, Pin_4,  TIM_Channel_1, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource4,  GPIO_AF_2,  0},  // RC_CH5 - PB4  - *TIM3_CH1
    { TIM3,  GPIOB, Pin_5,  TIM_Channel_2, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource5,  GPIO_AF_2,  0},  // RC_CH6 - PB5  - *TIM3_CH2
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource0,  GPIO_AF_2,  0},  // RC_CH7 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP, GPIO_PinSource1,  GPIO_AF_2,  0},  // RC_CH8 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM16, GPIOA, Pin_6,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP, GPIO_PinSource6,  GPIO_AF_1,  0},  // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, GPIOA, Pin_7,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP, GPIO_PinSource7,  GPIO_AF_1,  0},  // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM4,  GPIOA, Pin_11, TIM_Channel_1, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource11, GPIO_AF_10, 0},  // PWM3 - PA11
    { TIM4,  GPIOA, Pin_12, TIM_Channel_2, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource12, GPIO_AF_10, 0},  // PWM4 - PA12
    { TIM4,  GPIOB, Pin_8,  TIM_Channel_3, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_2,  0},  // PWM5 - PB8
    { TIM4,  GPIOB, Pin_9,  TIM_Channel_4, TIM4_IRQn,               1, Mode_AF_PP, GPIO_PinSource9,  GPIO_AF_2,  0},  // PWM6 - PB9
    { TIM15, GPIOA, Pin_2,  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource2,  GPIO_AF_9,  0},  // PWM7 - PA2
    { TIM15, GPIOA, Pin_3,  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, Mode_AF_PP, GPIO_PinSource3,  GPIO_AF_9,  0},  // PWM8 - PA3
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP, GPIO_PinSource8,  GPIO_AF_6,  0},  // GPIO_TIMER / LED_STRIP
};

