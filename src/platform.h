
#pragma once

#ifdef STM32F3DISCOVERY

#include "stm32f30x_conf.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"

// FIXME use correct ID
#define U_ID_0 (*(uint32_t*)0x10000003)
#define U_ID_1 (*(uint32_t*)0x10000000)
#define U_ID_2 (*(uint32_t*)0x10000003)

#define LED0_GPIO   GPIOE
#define LED0_PIN    Pin_8|Pin_12 // Blue LEDs - PE8/PE12
#define LED0_INVERTED
#define LED1_GPIO   GPIOE
#define LED1_PIN    Pin_10|Pin_14  // Orange LEDs - PE10/PE14
#define LED1_INVERTED
#define BEEP_GPIO   GPIOE
#define BEEP_PIN    Pin_9|Pin_13 // Red LEDs - PE9/PE13
#define BUZZER_INVERTED
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

#define GYRO
#define ACC
#define BUZZER
#define LED0
#define LED1

#define SENSORS_SET (SENSOR_ACC)

#endif // STM32F3DISCOVERY

#ifdef STM32F10X_MD

#include "stm32f10x_conf.h"
#include "core_cm3.h"

// Chip Unique ID on F103
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

// Hardware definitions and GPIO
#ifdef FY90Q
 // FY90Q
#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_12
#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_15

#define GYRO
#define ACC
#define LED0
#define LED1

#define SENSORS_SET (SENSOR_ACC)

#else

#ifdef OLIMEXINO
// OLIMEXINO

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
// LED2 is using one of the pwm pins (PWM2), so we must not use PWM2.  @See pwmInit()
#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_1 // D3, PA1/USART2_RTS/ADC1/TIM2_CH3 - "LED2" on silkscreen, Yellow
#define LED0
#endif

#ifdef OLIMEXINO_UNCUT_LED1_E_JUMPER
#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_5 // D13, PA5/SPI1_SCK/ADC5 - "LED1" on silkscreen, Green
#define LED1
#endif

#define GYRO
#define ACC

#define SENSORS_SET (SENSOR_ACC)

#else
// Afroflight32

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

#define GYRO
#define ACC
#define MAG
#define BARO
#define LEDRING
#define SONAR
#define BUZZER
#define LED0
#define LED1

#define SENSORS_SET (SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)

#endif
#endif

#endif // STM32F10X_MD
