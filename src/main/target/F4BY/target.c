
#include <stdbool.h>
#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM16  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),          // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),    
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),     
    PWM8  | (MAP_TO_SERVO_OUTPUT << 8),         // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    PWM13 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    PWM16 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #4 or #6
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM11  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM12 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT << 8),
    PWM15  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM16  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM5  | (MAP_TO_SERVO_OUTPUT << 8),      // Swap to servo if needed
    PWM4  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM3  | (MAP_TO_SERVO_OUTPUT << 8),
    PWM2  | (MAP_TO_SERVO_OUTPUT << 8),
    0xFFFF
};

const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),          // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),    
    PWM7  | (MAP_TO_PWM_INPUT << 8),     
    PWM8  | (MAP_TO_PWM_INPUT << 8),         // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),     // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #2
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #3
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #4
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #5
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),     // servo #6
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM3, IO_TAG(PC9), TIM_Channel_4, TIM3_IRQn, 0, IOCFG_AF_PP,  GPIO_AF_TIM3},       // S1_IN
    { TIM3, IO_TAG(PC8), TIM_Channel_3, TIM3_IRQn, 0, IOCFG_AF_PP,  GPIO_AF_TIM3},      // S2_IN
    { TIM3, IO_TAG(PC6), TIM_Channel_1, TIM3_IRQn, 0, IOCFG_AF_PP,  GPIO_AF_TIM3},      // S3_IN
    { TIM3, IO_TAG(PC7), TIM_Channel_2, TIM3_IRQn, 0, IOCFG_AF_PP,  GPIO_AF_TIM3},      // S4_IN
    { TIM4, IO_TAG(PD15), TIM_Channel_4, TIM4_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S5_IN
    { TIM4, IO_TAG(PD14), TIM_Channel_3, TIM4_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S6_IN
    { TIM4, IO_TAG(PD13), TIM_Channel_2, TIM4_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S7_IN
    { TIM4, IO_TAG(PD12), TIM_Channel_1, TIM4_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM4}, // S8_IN

    { TIM2, IO_TAG(PA0), TIM_Channel_1, TIM2_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM2},    // S1_OUT
    { TIM2, IO_TAG(PA1), TIM_Channel_2, TIM2_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM2},    // S2_OUT
    { TIM5, IO_TAG(PA2), TIM_Channel_3, TIM5_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM5},    // S3_OUT
    { TIM5, IO_TAG(PA3), TIM_Channel_4, TIM5_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM5},    // S4_OUT
    { TIM1, IO_TAG(PE9), TIM_Channel_1, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1},    // S5_OUT
    { TIM1, IO_TAG(PE11), TIM_Channel_2, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1},    // S6_OUT
    { TIM1, IO_TAG(PE13), TIM_Channel_3, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1},    // S7_OUT
    { TIM1, IO_TAG(PE14), TIM_Channel_4, TIM1_CC_IRQn, 1, IOCFG_AF_PP, GPIO_AF_TIM1},    // S8_OUT
   
    { TIM9, IO_TAG(PE6), TIM_Channel_2, TIM1_BRK_TIM9_IRQn, 0, IOCFG_AF_PP, GPIO_AF_TIM9 },          // sonar echo if needed
};

