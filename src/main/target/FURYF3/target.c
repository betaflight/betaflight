
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  IO_TAG(PB3), TIM_Channel_2, 0, IOCFG_AF_PP,  GPIO_AF_1,    TIM_USE_PPM | TIM_USE_PWM }, // PPM IN

    { TIM4,  IO_TAG(PB7), TIM_Channel_2, 1, IOCFG_AF_PP,  GPIO_AF_2,    TIM_USE_MC_MOTOR }, // PWM4 - S1
    { TIM4,  IO_TAG(PB6), TIM_Channel_1, 1, IOCFG_AF_PP,  GPIO_AF_2,    TIM_USE_MC_MOTOR }, // PWM5 - S2
    { TIM17, IO_TAG(PB5), TIM_Channel_1, 1, IOCFG_AF_PP,  GPIO_AF_10,   TIM_USE_MC_MOTOR }, // PWM6 - S3
    { TIM16, IO_TAG(PB4), TIM_Channel_1, 1, IOCFG_AF_PP,  GPIO_AF_1,    TIM_USE_MC_MOTOR }, // PWM7 - S4

    { TIM3,  IO_TAG(PB0), TIM_Channel_3, 1, IOCFG_AF_PP,  GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_MOTOR }, // SS1 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  IO_TAG(PB1), TIM_Channel_4, 1, IOCFG_AF_PP,  GPIO_AF_2,    TIM_USE_PWM | TIM_USE_MC_MOTOR }, // SS1 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N

    { TIM1,  IO_TAG(PA8), TIM_Channel_1, 1, IOCFG_AF_PP,  GPIO_AF_6,    TIM_USE_ANY }, // GPIO TIMER - LED_STRIP

};
