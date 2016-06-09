
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), // PPM input
    
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8), // PPM input
    
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),   
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  GPIOB, Pin_3,  TIM_Channel_2, TIM2_IRQn,               0, Mode_AF_PP,      GPIO_PinSource3,  GPIO_AF_1}, // PPM IN
    { TIM3,  GPIOB, Pin_0,  TIM_Channel_3, TIM3_IRQn,               0, Mode_AF_PP,      GPIO_PinSource0,  GPIO_AF_2}, // SS1 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  GPIOB, Pin_1,  TIM_Channel_4, TIM3_IRQn,               0, Mode_AF_PP,      GPIO_PinSource1,  GPIO_AF_2}, // SS1 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM4,  GPIOB, Pin_7,  TIM_Channel_2, TIM4_IRQn,               1, Mode_AF_PP,      GPIO_PinSource7,  GPIO_AF_2}, // PWM4 - S1
    { TIM4,  GPIOB, Pin_6,  TIM_Channel_1, TIM4_IRQn,               1, Mode_AF_PP,      GPIO_PinSource6,  GPIO_AF_2}, // PWM5 - S2
    { TIM17, GPIOB, Pin_5,  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, 1, Mode_AF_PP,      GPIO_PinSource5,  GPIO_AF_10}, // PWM6 - S3
    { TIM16, GPIOB, Pin_4,  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      1, Mode_AF_PP,      GPIO_PinSource4,  GPIO_AF_1}, // PWM7 - S4
 
    { TIM1,  GPIOA, Pin_8,  TIM_Channel_1, TIM1_CC_IRQn,            1, Mode_AF_PP,      GPIO_PinSource8,  GPIO_AF_6}, // GPIO TIMER - LED_STRIP

};


