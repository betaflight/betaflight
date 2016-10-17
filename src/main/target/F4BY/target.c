
#include <stdbool.h>
#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3, IO_TAG(PC9),  TIM_Channel_4, TIM3_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM3}, // S1_IN
    { TIM3, IO_TAG(PC8),  TIM_Channel_3, TIM3_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM3}, // S2_IN
    { TIM3, IO_TAG(PC6),  TIM_Channel_1, TIM3_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM3}, // S3_IN
    { TIM3, IO_TAG(PC7),  TIM_Channel_2, TIM3_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM3}, // S4_IN
    { TIM4, IO_TAG(PD15), TIM_Channel_4, TIM4_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S5_IN
    { TIM4, IO_TAG(PD14), TIM_Channel_3, TIM4_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S6_IN
    { TIM4, IO_TAG(PD13), TIM_Channel_2, TIM4_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S7_IN
    { TIM4, IO_TAG(PD12), TIM_Channel_1, TIM4_IRQn,    0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S8_IN

    { TIM2, IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,    1, IOCFG_AF_PP, GPIO_AF_TIM2}, // S1_OUT
    { TIM2, IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,    1, IOCFG_AF_PP, GPIO_AF_TIM2}, // S2_OUT
    { TIM5, IO_TAG(PA2),  TIM_Channel_3, TIM5_IRQn,    1, IOCFG_AF_PP, GPIO_AF_TIM5}, // S3_OUT
    { TIM5, IO_TAG(PA3),  TIM_Channel_4, TIM5_IRQn,    1, IOCFG_AF_PP, GPIO_AF_TIM5}, // S4_OUT
    { TIM1, IO_TAG(PE9),  TIM_Channel_1, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1}, // S5_OUT
    { TIM1, IO_TAG(PE11), TIM_Channel_2, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1}, // S6_OUT
    { TIM1, IO_TAG(PE13), TIM_Channel_3, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1}, // S7_OUT
    { TIM1, IO_TAG(PE14), TIM_Channel_4, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1}, // S8_OUT
   
    { TIM9, IO_TAG(PE6), TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM9 },          // sonar echo if needed
};

