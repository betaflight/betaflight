/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define IOCFG_OUT_PP        0
#define IOCFG_OUT_OD        0
#define IOCFG_AF_PP         0
#define IOCFG_AF_OD         0
#define IOCFG_IPD           0
#define IOCFG_IPU           0
#define IOCFG_IN_FLOATING   0

#define SPIDEV_COUNT        0

// no serial pins are defined for the simulator
#define SERIAL_TRAIT_PIN_CONFIG 0

#define I2CDEV_COUNT        0

#define RUN_LOOP_DELAY_MS 50 // max 20khz run loop frequency
#define USE_MAIN_ARGS
#define GYRO_COUNT 1 // 1 Gyro

typedef void* ADC_TypeDef; // Dummy definition for ADC_TypeDef
