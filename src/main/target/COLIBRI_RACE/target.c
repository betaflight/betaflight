
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

const uint16_t multiPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),			// PPM input
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

const uint16_t airPWM[] = {
    // prevent crashing, but do nothing
    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),			// Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM10  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    PWM11  | (MAP_TO_MOTOR_OUTPUT << 8),      	// Swap to servo if needed
    0xFFFF
};

const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_6, 0}, // PWM1 - PA8
    { TIM3,  IO_TAG(PC6),  TIM_Channel_1, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PWM2 - PC6
    { TIM3,  IO_TAG(PC7),  TIM_Channel_2, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PWM3 - PC7
    { TIM3,  IO_TAG(PC8),  TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PMW4 - PC8
    { TIM3,  IO_TAG(PC9),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0}, // PWM5 - PC9
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // PWM6 - PA0
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // PWM7 - PA1
    { TIM2,  IO_TAG(PA2),  TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // PWM8 - PA2
    { TIM2,  IO_TAG(PA3),  TIM_Channel_4, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // PWM9 - PA3
    { TIM15, IO_TAG(PB14), TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP_PD, GPIO_AF_1, 0}, // PWM10 - PB14
    { TIM15, IO_TAG(PB15), TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP_PD, GPIO_AF_1, 0}, // PWM11 - PB15
};

