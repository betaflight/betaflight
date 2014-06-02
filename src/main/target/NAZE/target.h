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
