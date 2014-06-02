// OLIMEXINO

//#define OLIMEXINO_UNCUT_LED1_E_JUMPER
//#define OLIMEXINO_UNCUT_LED2_E_JUMPER

#ifdef OLIMEXINO_UNCUT_LED1_E_JUMPER
#define LED0_GPIO   GPIOA
#define LED0_PIN    Pin_5 // D13, PA5/SPI1_SCK/ADC5 - "LED1" on silkscreen, Green
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED0
#endif

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
// "LED2" is using one of the PWM pins (CH2/PWM2), so we must not use PWM2 unless the jumper is cut.  @See pwmInit()
#define LED1_GPIO   GPIOA
#define LED1_PIN    Pin_1 // D3, PA1/USART2_RTS/ADC1/TIM2_CH3 - "LED2" on silkscreen, Yellow
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED1
#endif

#define GYRO
#define ACC

#define SENSORS_SET (SENSOR_ACC)
