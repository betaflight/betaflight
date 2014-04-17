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

#include "common/axis.h"

#include "drivers/accgyro_common.h"
#include "drivers/gpio_common.h"
#include "drivers/system_common.h"
#include "drivers/barometer_common.h"
#include "sensors_common.h"

#include "platform.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "boardalignment.h"
#include "battery.h"

#ifdef FY90Q
 // FY90Q
#include "drivers/accgyro_fy90q.h"
#include "drivers/adc_common.h"
#include "drivers/adc_fy90q.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/pwm_common.h"
#include "drivers/timer_common.h"
#include "drivers/serial_common.h"
#include "drivers/serial_uart.h"
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
#include "drivers/barometer_bmp085.h"
#include "drivers/barometer_ms5611.h"
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
