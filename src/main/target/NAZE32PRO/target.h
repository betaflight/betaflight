/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_12
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOB
#define BEEP_GPIO   GPIOB
#define BEEP_PIN    Pin_10
#define BEEP_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEPER
#define LED0

#define GYRO
#define ACC

#define SERIAL_PORT_COUNT 5

#define I2C_DEVICE (I2CDEV_1)

#define SENSORS_SET (SENSOR_ACC)

#define GPS
#define LED_STRIP
#define TELEMETRY
#define SOFT_SERIAL
#define SERIAL_RX
#define AUTOTUNE
