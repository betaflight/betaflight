/*
 * This file is deprecated.  All this code belongs elsewhere - create appropriate headers and include them.
 */

#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#define __USE_C99_MATH // for roundf()
#include <math.h>

#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "platform.h"
#include "drivers/accgyro_common.h"
#include "drivers/gpio_common.h"
#include "drivers/system_common.h"
#include "sensors_common.h"

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_NONE = 5
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SERIALRX = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
    FEATURE_VARIO = 1 << 13,
    FEATURE_3D = 1 << 14,
} AvailableFeatures;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
} SerialRXType;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK_NMEA,
    GPS_MTK_BINARY,
    GPS_MAG_BINARY,
    GPS_HARDWARE_MAX = GPS_MAG_BINARY,
} GPSHardware;

typedef enum {
    GPS_BAUD_115200 = 0,
    GPS_BAUD_57600,
    GPS_BAUD_38400,
    GPS_BAUD_19200,
    GPS_BAUD_9600,
    GPS_BAUD_MAX = GPS_BAUD_9600
} GPSBaudRates;

typedef enum {
    TELEMETRY_PROVIDER_FRSKY = 0,
    TELEMETRY_PROVIDER_HOTT,
    TELEMETRY_PROVIDER_MAX = TELEMETRY_PROVIDER_HOTT
} TelemetryProvider;

typedef enum {
    TELEMETRY_PORT_UART = 0,
    TELEMETRY_PORT_SOFTSERIAL_1, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_SOFTSERIAL_2, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_MAX = TELEMETRY_PORT_SOFTSERIAL_2
} TelemetryPort;

enum {
    GYRO_UPDATED = 1 << 0,
    ACC_UPDATED = 1 << 1,
    MAG_UPDATED = 1 << 2,
    TEMP_UPDATED = 1 << 3
};

typedef struct sensor_data_t
{
    int16_t gyro[3];
    int16_t acc[3];
    int16_t mag[3];
    float temperature;
    int updated;
} sensor_data_t;

typedef void (* baroOpFuncPtr)(void);                       // baro start operation
typedef void (* baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)
typedef uint16_t (* rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (* pidControllerFuncPtr)(void);                // pid controller function prototype

typedef struct baro_t
{
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

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

// Helpful macros
#ifdef LED0
#define LED0_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED0_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED0_ON                  digitalLo(LED0_GPIO, LED0_PIN);
#else
#define LED0_TOGGLE
#define LED0_OFF
#define LED0_ON
#endif

#ifdef LED1
#define LED1_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED1_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED1_ON                  digitalLo(LED1_GPIO, LED1_PIN);
#else
#define LED1_TOGGLE
#define LED1_OFF
#define LED1_ON
#endif

#ifdef BEEP_GPIO
#define BEEP_TOGGLE              digitalToggle(BEEP_GPIO, BEEP_PIN);
#define BEEP_OFF                 systemBeep(false);
#define BEEP_ON                  systemBeep(true);
#else
#define BEEP_TOGGLE              ;
#define BEEP_OFF                 ;
#define BEEP_ON                  ;
#endif

#undef SOFT_I2C                 // enable to test software i2c

#include "boardalignment.h"
#include "battery.h"
#include "math.h"

#ifdef FY90Q
 // FY90Q
#include "drivers/adc/drv_adc.h"
#include "drv_i2c.h"
#include "drv_pwm.h"
#include "drivers/serial/drv_uart.h"
#else

#ifdef OLIMEXINO
// OLIMEXINO
#include "drivers/adc_common.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_l3g4200d.h"
#include "drivers/pwm_common.h"
#include "drivers/timer_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_softserial.h"
#else

 // AfroFlight32
#include "drivers/adc_common.h"
#include "drivers/accgyro_adxl345.h"
#include "drivers/accgyro_bma280.h"
#include "drivers/altimeter_bmp085.h"
#include "drivers/altimeter_ms5611.h"
#include "drivers/compass_hmc5883l.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/light_ledring.h"
#include "drivers/accgyro_mma845x.h"
#include "drivers/accgyro_mpu3050.h"
#include "drivers/accgyro_mpu6050.h"
#include "drivers/accgyro_l3g4200d.h"
#include "drivers/pwm_common.h"
#include "drivers/timer_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_softserial.h"
#include "drivers/sonar_hcsr04.h"

#endif
#endif
