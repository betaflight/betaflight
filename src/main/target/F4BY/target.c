
#include <stdbool.h>
#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3, IO_TAG(PC9), TIM_Channel_4, 0, IOCFG_AF_PP,  GPIO_AF_TIM3,  TIM_USE_PPM | TIM_USE_PWM },     // S1_IN
    { TIM3, IO_TAG(PC8), TIM_Channel_3, 0, IOCFG_AF_PP,  GPIO_AF_TIM3,  TIM_USE_PWM },     // S2_IN
    { TIM3, IO_TAG(PC6), TIM_Channel_1, 0, IOCFG_AF_PP,  GPIO_AF_TIM3,  TIM_USE_PWM },     // S3_IN
    { TIM3, IO_TAG(PC7), TIM_Channel_2, 0, IOCFG_AF_PP,  GPIO_AF_TIM3,  TIM_USE_PWM },     // S4_IN
    { TIM4, IO_TAG(PD15), TIM_Channel_4, 0, IOCFG_AF_PP, GPIO_AF_TIM4,  TIM_USE_PWM },     // S5_IN
    { TIM4, IO_TAG(PD14), TIM_Channel_3, 0, IOCFG_AF_PP, GPIO_AF_TIM4,  TIM_USE_PWM },     // S6_IN
    { TIM4, IO_TAG(PD13), TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_TIM4,  TIM_USE_PWM },     // S7_IN
    { TIM4, IO_TAG(PD12), TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_TIM4,  TIM_USE_PWM },     // S8_IN

    { TIM2, IO_TAG(PA0), TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_TIM2,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR },    // S1_OUT
    { TIM2, IO_TAG(PA1), TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_TIM2,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_MOTOR },    // S2_OUT
    { TIM5, IO_TAG(PA2), TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_TIM5,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S3_OUT
    { TIM5, IO_TAG(PA3), TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_TIM5,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S4_OUT
    { TIM1, IO_TAG(PE9), TIM_Channel_1, 1, IOCFG_AF_PP, GPIO_AF_TIM1,   TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S5_OUT
    { TIM1, IO_TAG(PE11), TIM_Channel_2, 1, IOCFG_AF_PP, GPIO_AF_TIM1,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S6_OUT
    { TIM1, IO_TAG(PE13), TIM_Channel_3, 1, IOCFG_AF_PP, GPIO_AF_TIM1,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S7_OUT
    { TIM1, IO_TAG(PE14), TIM_Channel_4, 1, IOCFG_AF_PP, GPIO_AF_TIM1,  TIM_USE_MC_MOTOR |                    TIM_USE_FW_SERVO},    // S8_OUT

    { TIM9, IO_TAG(PE6), TIM_Channel_2, 0, IOCFG_AF_PP, GPIO_AF_TIM9,   TIM_USE_ANY },          // HC-SR04 echo if needed
};

