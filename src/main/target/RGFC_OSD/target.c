
#include <stdbool.h>
#include <stdint.h>

#include <platform.h>
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"

static const uint16_t multiPPM[] = {
    PWM4  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
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

static const uint16_t airPPM[] = {
    PWM4  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM4  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM1  | (MAP_TO_PWM_INPUT << 8),
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
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
#if defined(RGFC_OSD)
const timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    //
    // PPM PORT - Also USART2 RX
    //
    { TIM2, GPIOA, Pin_0, TIM_Channel_1, TIM2_IRQn,    0, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_1},    // PWM1  - RC1
    { TIM2, GPIOA, Pin_1, TIM_Channel_2, TIM2_IRQn,    0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_1},    // PWM2  - RC2
    { TIM2, GPIOA, Pin_2, TIM_Channel_3, TIM2_IRQn,    0, Mode_AF_PP, GPIO_PinSource2, GPIO_AF_1},    // PWM3  - RC3
    { TIM2, GPIOA, Pin_3, TIM_Channel_4, TIM2_IRQn,    0, Mode_AF_PP, GPIO_PinSource3, GPIO_AF_1},    // PWM4  - RC4
    { TIM3, GPIOA, Pin_6, TIM_Channel_1, TIM3_IRQn,    0, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2},    // PWM5  - RC5
    { TIM3, GPIOA, Pin_7, TIM_Channel_2, TIM3_IRQn,    0, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_2},    // PWM6  - RC6
    { TIM3, GPIOB, Pin_0, TIM_Channel_3, TIM3_IRQn,    0, Mode_AF_PP, GPIO_PinSource0, GPIO_AF_2},    // PWM7  - RC7
    { TIM3, GPIOB, Pin_1, TIM_Channel_4, TIM3_IRQn,    0, Mode_AF_PP, GPIO_PinSource1, GPIO_AF_2},    // PWM8  - RC8

    // Main outputs 8 PWM

    { TIM4, GPIOB, Pin_6, TIM_Channel_1, TIM4_IRQn,    1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_2},
    { TIM4, GPIOB, Pin_7, TIM_Channel_2, TIM4_IRQn,    1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_2},
    { TIM4, GPIOB, Pin_8, TIM_Channel_3, TIM4_IRQn,    1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_2},
    { TIM4, GPIOB, Pin_9, TIM_Channel_4, TIM4_IRQn,    1, Mode_AF_PP, GPIO_PinSource9, GPIO_AF_2},
    { TIM8, GPIOC, Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource6, GPIO_AF_4},
    { TIM8, GPIOC, Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource7, GPIO_AF_4},
    { TIM8, GPIOC, Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource8, GPIO_AF_4},
    { TIM8, GPIOC, Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 1, Mode_AF_PP, GPIO_PinSource9, GPIO_AF_4},

};

#define USED_TIMERS  (TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(8))

#define TIMER_APB1_PERIPHERALS ( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 )
#define TIMER_APB2_PERIPHERALS ( RCC_APB2Periph_TIM8 )
#define TIMER_AHB_PERIPHERALS (RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC )
#endif
t timerHardware_t timerHardware[USABLE_TIMER_CHANNEL_COUNT] = {
    // PPM / UART2 RX
    { TIM8,  IO_TAG(PA15), TIM_Channel_1, TIM8_CC_IRQn,            0, IOCFG_AF_PP_PD, GPIO_AF_2, 0},  // PPM
    { TIM2,  IO_TAG(PA0),  TIM_Channel_1, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0},  // PWM1
    { TIM2,  IO_TAG(PA1),  TIM_Channel_2, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0},  // PWM2
    { TIM15, IO_TAG(PA2),  TIM_Channel_1, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_9, 0},  // PWM3
    { TIM15, IO_TAG(PA3),  TIM_Channel_2, TIM1_BRK_TIM15_IRQn,     1, IOCFG_AF_PP,    GPIO_AF_9, 0},  // PWM4
    { TIM3,  IO_TAG(PA6),  TIM_Channel_1, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0},  // PWM5
    { TIM3,  IO_TAG(PA7),  TIM_Channel_2, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0},  // PWM6
    { TIM3,  IO_TAG(PB0),  TIM_Channel_3, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0},  // PWM7
    { TIM3,  IO_TAG(PB1),  TIM_Channel_4, TIM3_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_2, 0},  // PWM8
    { TIM2,  IO_TAG(PB10), TIM_Channel_3, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // RC_CH4 - PB10 - *TIM2_CH3, USART3_TX (AF7)
    { TIM2,  IO_TAG(PB11), TIM_Channel_4, TIM2_IRQn,               1, IOCFG_AF_PP,    GPIO_AF_1, 0}, // RC_CH3 - PB11 - *TIM2_CH4, USART3_RX (AF7)
    { TIM1,  IO_TAG(PA8),  TIM_Channel_1, TIM1_CC_IRQn,            1, IOCFG_AF_PP,    GPIO_AF_6, 0},  // GPIO_TIMER / LED_STRIP
};

