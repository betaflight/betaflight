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
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED0_PERIPHERAL RCC_APB2Periph_GPIOA
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define LED1_PERIPHERAL RCC_APB2Periph_GPIOB


#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define BEEP_PERIPHERAL RCC_APB2Periph_GPIOA

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
