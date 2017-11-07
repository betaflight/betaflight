
#include <stdbool.h>
#include <platform.h>
#include "drivers/io.h"

#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/timer_def.h"

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    DEF_TIM(TIM3, CH4, PC9,  TIM_USE_PWM,   0, 0), // S1_IN
    DEF_TIM(TIM3, CH3, PC8,  TIM_USE_PWM,   0, 0), // S2_IN
    DEF_TIM(TIM3, CH1, PC6,  TIM_USE_PWM,   0, 0), // S3_IN
    DEF_TIM(TIM3, CH2, PC7,  TIM_USE_PWM,   0, 0), // S4_IN
    DEF_TIM(TIM4, CH4, PD15, TIM_USE_PWM,   0, 0), // S5_IN
    DEF_TIM(TIM4, CH3, PD14, TIM_USE_PWM,   0, 0), // S6_IN
    DEF_TIM(TIM4, CH2, PD13, TIM_USE_PWM,   0, 0), // S7_IN
    DEF_TIM(TIM4, CH1, PD12, TIM_USE_PWM,   0, 0), // S8_IN
    DEF_TIM(TIM2, CH1, PA0,  TIM_USE_MOTOR, 0, 0), // S1_OUT
    DEF_TIM(TIM2, CH2, PA1,  TIM_USE_MOTOR, 0, 0), // S2_OUT
    DEF_TIM(TIM5, CH3, PA2,  TIM_USE_MOTOR, 0, 0), // S3_OUT
    DEF_TIM(TIM5, CH4, PA3,  TIM_USE_MOTOR, 0, 0), // S4_OUT
    DEF_TIM(TIM1, CH1, PE9,  TIM_USE_MOTOR, 0, 0), // S5_OUT
    DEF_TIM(TIM1, CH2, PE11, TIM_USE_MOTOR, 0, 0), // S6_OUT
    DEF_TIM(TIM1, CH3, PE13, TIM_USE_MOTOR, 0, 0), // S7_OUT
    DEF_TIM(TIM1, CH4, PE14, TIM_USE_MOTOR, 0, 0), // S8_OUT
    DEF_TIM(TIM9, CH2, PE6,  TIM_USE_MOTOR, 0, 0), // sonar echo if needed
};
