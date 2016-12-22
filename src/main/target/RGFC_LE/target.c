
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"

#include "drivers/timer.h"

/*
const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

const uint16_t multiPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
	0xFFFF
};

const uint16_t airPPM[] = {
	PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM11 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM12 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
	0xFFFF
};

const uint16_t airPWM[] = {
	PWM1  | (MAP_TO_PWM_INPUT    << 8), // PPM input

	PWM2  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM3  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM4  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),
	PWM10 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM11 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM12 | (MAP_TO_SERVO_OUTPUT << 8),
	PWM13 | (MAP_TO_SERVO_OUTPUT << 8),
	0xFFFF
};
*/
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2, IO_TAG(PA3), TIM_Channel_4, TIM2_IRQn,    TIM_USE_PWM | TIM_USE_PPM, 0, GPIO_AF_1 },
    // Main outputs 6 PWM
    { TIM4, IO_TAG(PB6), TIM_Channel_1, TIM4_IRQn,    TIM_USE_MOTOR,             1, GPIO_AF_2 },
    { TIM4, IO_TAG(PB7), TIM_Channel_2, TIM4_IRQn,    TIM_USE_MOTOR,             1, GPIO_AF_2 },
    { TIM4, IO_TAG(PB8), TIM_Channel_3, TIM4_IRQn,    TIM_USE_MOTOR,             1, GPIO_AF_2 },
    { TIM4, IO_TAG(PB9), TIM_Channel_4, TIM4_IRQn,    TIM_USE_MOTOR,             1, GPIO_AF_2 },
    { TIM8, IO_TAG(PC6), TIM_Channel_1, TIM8_CC_IRQn, TIM_USE_MOTOR,             1, GPIO_AF_4 },
    { TIM8, IO_TAG(PC7), TIM_Channel_2, TIM8_CC_IRQn, TIM_USE_MOTOR,             1, GPIO_AF_4 },
    // Servo outputs 2 PWM
    { TIM3, IO_TAG(PC6), TIM_Channel_1, TIM3_IRQn,    TIM_USE_SERVO,             1, GPIO_AF_2 },
    { TIM3, IO_TAG(PC7), TIM_Channel_2, TIM3_IRQn,    TIM_USE_SERVO,             1, GPIO_AF_2 },
	// Additional outputs
    { TIM16, IO_TAG(PA6), TIM_Channel_1, TIM1_UP_TIM16_IRQn,       TIM_USE_LED,  1, GPIO_AF_1 },
    { TIM17, IO_TAG(PA7), TIM_Channel_2, TIM1_TRG_COM_TIM17_IRQn,  TIM_USE_LED,  1, GPIO_AF_1 },
};

/*
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               TIM_USE_PWM | TIM_USE_PPM, 0, GPIO_AF_1 }, // RC_CH1 - PA0  - *TIM2_CH1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               TIM_USE_PWM,   0, GPIO_AF_1 }, // RC_CH2 - PA1  - *TIM2_CH2, TIM15_CH1N
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               TIM_USE_PWM,   0, GPIO_AF_1 }, // RC_CH3 - PB11 - *TIM2_CH4, UART3_RX (AF7)
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM2_IRQn,               TIM_USE_PWM,   0, GPIO_AF_1 }, // RC_CH4 - PB10 - *TIM2_CH3, UART3_TX (AF7)
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               TIM_USE_PWM,   0, GPIO_AF_2 }, // RC_CH5 - PB0  - *TIM3_CH3, TIM1_CH2N, TIM8_CH2N
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               TIM_USE_PWM,   0, GPIO_AF_2 }, // RC_CH6 - PB1  - *TIM3_CH4, TIM1_CH3N, TIM8_CH3N
    { TIM16, IO_TAG(PA6),  TIM_Channel_1, TIM1_UP_TIM16_IRQn,      TIM_USE_MOTOR, 1, GPIO_AF_1 }, // PWM1 - PA6  - TIM3_CH1, TIM8_BKIN, TIM1_BKIN, *TIM16_CH1
    { TIM17, IO_TAG(PA7),  TIM_Channel_1, TIM1_TRG_COM_TIM17_IRQn, TIM_USE_MOTOR, 1, GPIO_AF_1 }, // PWM2 - PA7  - TIM3_CH2, *TIM17_CH1, TIM1_CH1N, TIM8_CH1
    { TIM4,  IO_TAG(PA11), TIM_Channel_1, TIM4_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_10 },// PWM3 - PA11
    { TIM4,  IO_TAG(PA12), TIM_Channel_2, TIM4_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_10 },// PWM4 - PA12
    { TIM4,  IO_TAG(PB8),  TIM_Channel_3, TIM4_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM5 - PB8
    { TIM4,  IO_TAG(PB9),  TIM_Channel_4, TIM4_IRQn,               TIM_USE_MOTOR, 1, GPIO_AF_2 }, // PWM6 - PB9
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     TIM_USE_MOTOR, 1, GPIO_AF_9 }, // PWM7 - PA2
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     TIM_USE_MOTOR, 1, GPIO_AF_9 }, // PWM8 - PA3
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            TIM_USE_MOTOR, 1, GPIO_AF_6 }, // GPIO_TIMER / LED_STRIP
};
*/

