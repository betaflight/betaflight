#pragma once

// for roundf()
#define __USE_C99_MATH

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x_conf.h"
#include "core_cm3.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

typedef enum {
    SENSOR_ACC = 1 << 0,
    SENSOR_BARO = 1 << 1,
    SENSOR_MAG = 1 << 2,
    SENSOR_SONAR = 1 << 3,
    SENSOR_GPS = 1 << 4,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SPEKTRUM = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_GYRO_SMOOTHING = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
} AvailableFeatures;

typedef void (* sensorInitFuncPtr)(void);                   // sensor init prototype
typedef void (* sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef int32_t (* baroCalculateFuncPtr)(void);             // baro calculation (returns altitude in cm based on static data collected)
typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data

typedef struct sensor_t
{
    sensorInitFuncPtr init;
    sensorReadFuncPtr read;
    sensorReadFuncPtr align;
    sensorReadFuncPtr temperature;
} sensor_t;

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    uint16_t repeat_delay;
    sensorInitFuncPtr start_ut;
    sensorInitFuncPtr get_ut;
    sensorInitFuncPtr start_up;
    sensorInitFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }

// Hardware definitions and GPIO

#ifdef FY90Q
 // FY90Q
#define LED0_GPIO   GPIOC
#define LED0_PIN    GPIO_Pin_12
#define LED1_GPIO   GPIOA
#define LED1_PIN    GPIO_Pin_15

#define GYRO
#define ACC

#else
 // Afroflight32
#define LED0_GPIO   GPIOB
#define LED0_PIN    GPIO_Pin_3
#define LED1_GPIO   GPIOB
#define LED1_PIN    GPIO_Pin_4
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    GPIO_Pin_12
#define BARO_GPIO   GPIOC
#define BARO_PIN    GPIO_Pin_13

#define GYRO
#define ACC
#define MAG
#define BARO
#define LEDRING
#define SONAR

#endif

// Helpful macros
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);

#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 digitalHi(BEEP_GPIO, BEEP_PIN);
#define BEEP_ON                  digitalLo(BEEP_GPIO, BEEP_PIN);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

#undef SOFT_I2C                 // enable to test software i2c

#ifdef FY90Q
 // FY90Q
#include "drv_system.h"         // timers, delays, etc
#include "drv_adc.h"
#include "drv_i2c.h"
#include "drv_pwm.h"
#include "drv_uart.h"

#else
 // AfroFlight32
#include "drv_system.h"         // timers, delays, etc
#include "drv_adc.h"
#include "drv_adxl345.h"
#include "drv_bmp085.h"
#include "drv_ms5611.h"
#include "drv_hmc5883l.h"
#include "drv_i2c.h"
#include "drv_ledring.h"
#include "drv_mma845x.h"
#include "drv_mpu3050.h"
#include "drv_mpu6050.h"
#include "drv_l3g4200d.h"
#include "drv_pwm.h"
#include "drv_uart.h"
#include "drv_hcsr04.h"

#endif
